# Hardware-free verdict rules. Reuse the existing INFO/telemetry parser and
# conservative startup budget; do not load an orchestrator from host tests.
. "$PSScriptRoot/rxstall-verdicts.ps1"

function Assert-WatchdogModes([string[]]$Modes, [ValidateSet('none', 'wait', 'reset', 'abrupt', 'abrupt_idle')][string]$Recovery = 'none') {
    if ($null -eq $Modes -or $Modes.Count -eq 0) { throw 'Select at least one watchdog mode.' }
    $seen = @{}
    foreach ($mode in $Modes) {
        if ($mode -notin @('120', '140', '200')) { throw "Unsupported watchdog mode '$mode'." }
        if ($Recovery -ne 'none' -and $mode -eq '200') { throw 'Recovery diagnostics accept only the 120/140 ms fault cases.' }
        if ($seen.ContainsKey($mode)) { throw "Duplicate watchdog mode '$mode'; retries are not allowed in a run." }
        $seen[$mode] = $true
    }
}

function Get-WatchdogAggregateResult([string[]]$Results) {
    if ($null -eq $Results -or $Results.Count -eq 0) { throw 'An empty case list cannot pass.' }
    foreach ($result in $Results) {
        if ($result -notin @('pass', 'fail')) { throw "Unfinished or unknown case result '$result'." }
    }
    if ('fail' -in $Results) { return 'fail' }
    return 'pass'
}

function Get-WatchdogMarkers([ValidateSet('120', '140', '200')][string]$Mode) {
    $target = if ($Mode -eq '200') { 'exact 16 bytes and STOP' } else { 'held RX and explicit target abort' }
    return @{
        controller = "WATCHDOG $Mode CONTROLLER PASS"
        target = "WATCHDOG $Mode TARGET PASS: $target"
    }
}

function Assert-WatchdogMarker([string]$Log, [string]$Marker, [string]$FamilyPattern) {
    $plain = $Log -replace ([char]27 + '\[[0-9;]*m'), ''
    $start = '(?m)^[ \t]*(?:\d+(?:\.\d+)?[ \t]+)?\[INFO[ \t]*\][ \t]+'
    $candidates = [regex]::Matches($plain, $start + $FamilyPattern + '[^\r\n]*')
    if ($candidates.Count -ne 1 -or -not (Test-RxstallMarker $Log $Marker)) {
        throw "Expected exactly one exact '$Marker' in its record family; found $($candidates.Count)."
    }
}

function Assert-WatchdogEndpoint([ValidateSet('120', '140', '200')][string]$Mode,
    [ValidateSet('controller', 'target')][string]$Role, [int]$ExitCode, [string]$Log) {
    if ($ExitCode -ne 0) { throw "$Role failed: exit $ExitCode" }
    $markers = Get-WatchdogMarkers $Mode
    Assert-WatchdogMarker $Log $markers[$Role] ('WATCHDOG [^\r\n]* ' + $Role.ToUpperInvariant() + ' PASS')
}

function Get-WatchdogRecovery([ValidateSet('120', '140', '200')][string]$Mode, [string]$ControllerLog,
    [ValidateSet('none', 'wait', 'reset', 'abrupt', 'abrupt_idle')][string]$Recovery = 'none') {
    $plain = $ControllerLog -replace ([char]27 + '\[[0-9;]*m'), ''
    $family = '(?m)^[ \t]*(?:\d+(?:\.\d+)?[ \t]+)?\[INFO[ \t]*\][ \t]+WATCHDOG RECOVERY:[^\r\n]*'
    if ([regex]::Matches($plain, $family).Count -ne 2) {
        throw 'Expected exactly two WATCHDOG RECOVERY snapshots, one return and one after_wait; extras are not ignored.'
    }
    $snapshots = [ordered]@{}
    $previousIndex = -1
    foreach ($stage in @('return', 'after_wait')) {
        $record = Get-RxstallRecord $ControllerLog 'INFO' "WATCHDOG RECOVERY: stage=$stage," `
            ("WATCHDOG RECOVERY: stage=$stage, mcr=(?<mcr>[0-9]+), msr=(?<msr>[0-9]+), mfsr=(?<mfsr>[0-9]+), scl_high=(?<scl>true|false)")
        if ($record.Index -le $previousIndex) { throw 'WATCHDOG RECOVERY snapshots must appear return then after_wait.' }
        $previousIndex = $record.Index
        $mcr = [uint32]::Parse($record.Groups['mcr'].Value)
        $msr = [uint32]::Parse($record.Groups['msr'].Value)
        $mfsr = [uint32]::Parse($record.Groups['mfsr'].Value)
        $sclHigh = $record.Groups['scl'].Value -eq 'true'
        # Pinned MCXA LPI2C fields: MCR.MEN[0], MSR.MBF[24], and
        # MFSR.TXCOUNT[2:0]/RXCOUNT[18:16]. MBF is not the BBF bus-busy bit.
        $idle = ($msr -band 0x01000000) -eq 0
        if (($mcr -band 1) -ne 1) { throw "$stage snapshot requires MEN enabled." }
        if (($mfsr -band 0x00070007) -ne 0) { throw "$stage snapshot requires empty RX and TX FIFOs." }
        if ($Mode -eq '200' -and -not $idle) { throw "$stage completing control still has controller MBF set." }
        if ($Recovery -eq 'none' -and -not $idle) {
            throw "$stage unrepaired watchdog checkpoint requires the local controller engine idle."
        }
        if ($Recovery -ne 'none' -and $stage -eq 'after_wait' -and -not $idle) {
            throw 'Repaired controller must be idle after the matched observation wait.'
        }
        if ($Recovery -eq 'abrupt_idle' -and $stage -eq 'after_wait' -and ($msr -band 0x02000000) -ne 0) {
            throw 'Idle-detector repair must clear MSR.BBF by the after-wait checkpoint.'
        }
        if ($stage -eq 'after_wait' -and -not $sclHigh) { throw 'SCL must sample high after the observation wait.' }
        $snapshots[$stage] = [ordered]@{
            mcr = $mcr; msr = $msr; mfsr = $mfsr; scl_high = $sclHigh; controller_idle = $idle
        }
    }
    return [ordered]@{
        snapshots = $snapshots
        controller_idle_at_return = $snapshots['return'].controller_idle
        controller_idle_after_wait = $snapshots['after_wait'].controller_idle
        local_engine_recovery_qualified = ($Recovery -eq 'none')
        terminal_recovery_qualified = $false
        scope = if ($Recovery -eq 'none') {
            'No intervening diagnostic repair: local controller MBF must be idle both at return and after the wait, with SCL high after peer release. This qualifies only local-engine recovery; BBF may remain set and no subsequent transfer is attempted. Historical busy-engine logs remain preserved evidence but do not satisfy this strengthened acceptance rule.'
        } else {
            'The return snapshot precedes diagnostic intervention and may retain the historical busy-engine condition; after_wait follows the selected repair and must be idle. A passing external repair does not qualify production local-engine recovery or prove the original failure mechanism.'
        }
    }
}

function Get-WatchdogRepair([ValidateSet('none', 'wait', 'reset', 'abrupt', 'abrupt_idle')][string]$Recovery,
    [string]$ControllerLog, [string]$TargetLog) {
    if ($Recovery -eq 'none') {
        if ($ControllerLog.Contains('WATCHDOG REPAIR:') -or $TargetLog.Contains('WATCHDOG REPAIR:')) {
            throw 'Default characterization must contain no diagnostic repair records.'
        }
        return [ordered]@{ mode = 'none'; diagnostic_repair_qualified = $false }
    }
    $idleDetector = $Recovery -eq 'abrupt_idle'
    $controllerCount = if ($idleDetector) { 7 } else { 5 }
    foreach ($entry in @(@{ role = 'controller'; log = $ControllerLog; count = $controllerCount },
            @{ role = 'target'; log = $TargetLog; count = 2 })) {
        # Count every occurrence, not just valid INFO records. An unknown,
        # malformed or wrong-level repair cannot be hidden beside valid lines.
        if ([regex]::Matches($entry.log, 'WATCHDOG REPAIR:').Count -ne $entry.count) {
            throw "Expected exactly $($entry.count) repair records from $($entry.role); extras are not ignored."
        }
    }
    $controllerMode = Get-RxstallRecord $ControllerLog 'INFO' 'WATCHDOG REPAIR: mode=' `
        "WATCHDOG REPAIR: mode=$Recovery"
    $targetMode = Get-RxstallRecord $TargetLog 'INFO' 'WATCHDOG REPAIR: mode=' `
        "WATCHDOG REPAIR: mode=$Recovery"
    $abrupt = $Recovery -in @('abrupt', 'abrupt_idle')
    $waitField = if ($abrupt) { 'reset_wait_us' } else { 'disable_wait_us' }
    $waitLimit = if ($abrupt) { 1000 } else { 300000 }
    $acknowledgment = Get-RxstallRecord $ControllerLog 'INFO' "WATCHDOG REPAIR: $waitField=" `
        "WATCHDOG REPAIR: $waitField=(?<elapsed>[0-9]+)"
    $acknowledgmentUs = [uint64]::Parse($acknowledgment.Groups['elapsed'].Value)
    if ($acknowledgmentUs -gt $waitLimit) { throw "$waitField exceeded its $waitLimit us bound." }
    $idleConfiguration = $null
    $idleRecord = $null
    $linesRecord = $null
    if ($idleDetector) {
        $idleRecord = Get-RxstallRecord $ControllerLog 'INFO' 'WATCHDOG REPAIR: idle_detector ' `
            'WATCHDOG REPAIR: idle_detector busidle=(?<busidle>[0-9]+), clklo=(?<clklo>[0-9]+), sethold=(?<sethold>[0-9]+), clkhi=(?<clkhi>[0-9]+)'
        $idleConfiguration = [ordered]@{}
        foreach ($field in @('busidle', 'clklo', 'sethold', 'clkhi')) {
            $idleConfiguration[$field] = [uint32]::Parse($idleRecord.Groups[$field].Value)
            $maximum = if ($field -eq 'busidle') { 4095 } else { 63 }
            if ($idleConfiguration[$field] -gt $maximum) { throw "Idle-detector $field exceeds its register width." }
        }
        $expectedIdle = 2 * ($idleConfiguration.clklo + $idleConfiguration.sethold + 2)
        if ($idleConfiguration.busidle -ne $expectedIdle -or $idleConfiguration.busidle -le ($idleConfiguration.clkhi + 1)) {
            throw 'BUSIDLE must equal 2*(CLKLO+SETHOLD+2) and exceed CLKHI+1.'
        }
        $linesRecord = Get-RxstallRecord $ControllerLog 'INFO' 'WATCHDOG REPAIR: lines ' `
            'WATCHDOG REPAIR: lines scl_high=true, sda_high=true, bus_busy=false'
    }
    $configuration = Get-RxstallRecord $ControllerLog 'INFO' 'WATCHDOG REPAIR: configuration restored=' `
        'WATCHDOG REPAIR: configuration restored=true'
    $stage = if ($abrupt) { 'reset_complete' } else { 'disabled' }
    $expectedScl = if ($abrupt) { 'false' } else { 'true' }
    $expectedMen = if ($abrupt) { 1 } else { 0 }
    $checkpoint = Get-RxstallRecord $ControllerLog 'INFO' 'WATCHDOG REPAIR: stage=' `
        "WATCHDOG REPAIR: stage=$stage, mcr=(?<mcr>[0-9]+), msr=(?<msr>[0-9]+), mfsr=(?<mfsr>[0-9]+), scl_high=$expectedScl"
    $mcr = [uint32]::Parse($checkpoint.Groups['mcr'].Value)
    $msr = [uint32]::Parse($checkpoint.Groups['msr'].Value)
    $mfsr = [uint32]::Parse($checkpoint.Groups['mfsr'].Value)
    if (($mcr -band 3) -ne $expectedMen) { throw "$stage repair checkpoint requires MEN=$expectedMen and RST released." }
    if (($msr -band 0x01000000) -ne 0) { throw "$stage repair checkpoint requires controller MBF idle." }
    if (($mfsr -band 0x00070007) -ne 0) { throw "$stage repair checkpoint requires empty RX and TX FIFOs." }
    $controllerFresh = Get-RxstallRecord $ControllerLog 'INFO' 'WATCHDOG REPAIR: fresh ' `
        'WATCHDOG REPAIR: fresh write PASS'
    $targetFresh = Get-RxstallRecord $TargetLog 'INFO' 'WATCHDOG REPAIR: fresh ' `
        'WATCHDOG REPAIR: fresh exact 16 bytes and STOP'
    $returned = Get-RxstallRecord $ControllerLog 'INFO' 'WATCHDOG RECOVERY: stage=return,' `
        'WATCHDOG RECOVERY: stage=return, mcr=[0-9]+, msr=[0-9]+, mfsr=[0-9]+, scl_high=(true|false)'
    $afterWait = Get-RxstallRecord $ControllerLog 'INFO' 'WATCHDOG RECOVERY: stage=after_wait,' `
        'WATCHDOG RECOVERY: stage=after_wait, mcr=[0-9]+, msr=[0-9]+, mfsr=[0-9]+, scl_high=(true|false)'
    $sequence = @($returned, $controllerMode, $acknowledgment)
    if ($idleDetector) { $sequence += $idleRecord }
    $sequence += @($configuration, $checkpoint, $afterWait)
    if ($idleDetector) { $sequence += $linesRecord }
    $sequence += $controllerFresh
    $previous = -1
    foreach ($record in $sequence) {
        if ($record.Index -le $previous) { throw 'Controller repair records violate the required effect/evidence order.' }
        $previous = $record.Index
    }
    $targetReady = Get-RxstallRecord $TargetLog 'INFO' 'WATCHDOG TARGET READY:' 'WATCHDOG TARGET READY: (120|140)'
    $held = Get-RxstallRecord $TargetLog 'INFO' 'WATCHDOG TARGET: unserviced-RDF' `
        'WATCHDOG TARGET: unserviced-RDF samples=[0-9]+, elapsed_us=[0-9]+, RDDE/ERQ/ACTIVE off, SCL sampled=true'
    $released = Get-RxstallRecord $TargetLog 'INFO' 'WATCHDOG TARGET: SCL sampled high after release' `
        'WATCHDOG TARGET: SCL sampled high after release'
    $previous = -1
    foreach ($record in @($targetReady, $targetMode, $held, $released, $targetFresh)) {
        if ($record.Index -le $previous) { throw 'Target repair records violate the required hold/release/fresh-transfer order.' }
        $previous = $record.Index
    }
    return [ordered]@{
        mode = $Recovery; diagnostic_repair_qualified = $true
        $waitField = $acknowledgmentUs
        ($stage + '_checkpoint') = [ordered]@{ mcr = $mcr; msr = $msr; mfsr = $mfsr; scl_high = (-not $abrupt); controller_idle = $true }
        configuration_restored_or_unchanged = (-not $idleDetector)
        configuration_matches_declared_repair = $true
        diagnostic_configuration_change = if ($idleDetector) { 'MCFGR2.BUSIDLE enabled from MCCR0 timing readback.' } else { $null }
        bus_idle_configuration = $idleConfiguration
        both_lines_high_and_bus_not_busy_after_wait = if ($idleDetector) { $true } else { $null }
        fresh_exact_transfer_and_stop = $true
        scope = 'Matched post-return diagnostic repair followed by a new exact transfer. This does not identify stale RMW as the original cause or qualify the unchanged production recovery.'
    }
}

function Get-WatchdogObservations([ValidateSet('120', '140', '200')][string]$Mode,
    [string]$ControllerLog, [string]$TargetLog, [ValidateSet('none', 'wait', 'reset', 'abrupt', 'abrupt_idle')][string]$Recovery = 'none') {
    Assert-WatchdogModes @($Mode) $Recovery
    Assert-WatchdogMarker $ControllerLog 'WATCHDOG CONTROLLER: 20s quiet window' 'WATCHDOG CONTROLLER: 20s quiet window'
    Assert-WatchdogMarker $TargetLog "WATCHDOG TARGET READY: $Mode" 'WATCHDOG TARGET READY:'
    $config = Get-RxstallRecord $ControllerLog 'INFO' 'WATCHDOG CONFIG:' `
        'WATCHDOG CONFIG: requested_ms=(?<requested>[0-9]+), input_hz=(?<hz>[0-9]+), prescale=(?<prescale>[0-9]+), pinlow=(?<pinlow>[0-9]+), timecfg=(?<timecfg>[0-9]+), filtscl=(?<filter>[0-9]+), software_timeout_ms=(?<timeout>[0-9]+)'
    $expectedPinlow = @{ '120' = 1407; '140' = 1641; '200' = 2344 }[$Mode]
    $expected = @{
        requested = [uint32]$Mode; hz = 3000000; prescale = 0
        pinlow = $expectedPinlow; timecfg = 0; filter = 0; timeout = 500
    }
    $readback = @{}
    foreach ($field in $expected.Keys) {
        $readback[$field] = [uint32]::Parse($config.Groups[$field].Value)
        if ($readback[$field] -ne $expected[$field]) {
            throw "WATCHDOG $Mode config $field=$($readback[$field]), expected $($expected[$field])."
        }
    }
    $controller = Get-RxstallRecord $ControllerLog 'INFO' 'WATCHDOG CONTROLLER result=' `
        'WATCHDOG CONTROLLER result=(?<result>[^,\r\n]+), elapsed_us=(?<elapsed>[0-9]+)'
    $target = Get-RxstallRecord $TargetLog 'INFO' 'WATCHDOG TARGET: unserviced-RDF' `
        'WATCHDOG TARGET: unserviced-RDF samples=(?<samples>[0-9]+), elapsed_us=(?<elapsed>[0-9]+), RDDE/ERQ/ACTIVE off, SCL sampled=(?<sampled>true|false)'
    $controllerUs = [uint64]::Parse($controller.Groups['elapsed'].Value)
    $holdUs = [uint64]::Parse($target.Groups['elapsed'].Value)
    $samples = [uint32]::Parse($target.Groups['samples'].Value)
    $result = $controller.Groups['result'].Value
    $windows = @{ '120' = @(110000, 135000); '140' = @(130000, 155000); '200' = @(145000, 190000) }
    $window = $windows[$Mode]
    $expectedResult = if ($Mode -eq '200') { 'Ok(())' } else { 'Err(PinLowTimeout)' }
    if ($result -ne $expectedResult) { throw "$Mode controller result '$result', expected '$expectedResult'." }
    if ($controllerUs -lt $window[0] -or $controllerUs -gt $window[1]) {
        throw "$Mode controller elapsed_us=$controllerUs outside $($window[0])..$($window[1])."
    }
    if ($samples -lt 2) { throw "$Mode needs at least two unserviced-RDF samples." }
    if ($holdUs -lt 150000 -or $holdUs -gt 175000) {
        throw "$Mode target hold elapsed_us=$holdUs outside 150000..175000."
    }
    if ($target.Groups['sampled'].Value -ne 'true') { throw 'This experiment requires sampled SCL; unavailable is not a threshold pass.' }
    Assert-WatchdogMarker $TargetLog 'WATCHDOG TARGET: SCL sampled idle-high' 'WATCHDOG TARGET: SCL sampled idle-high'
    Assert-WatchdogMarker $TargetLog 'WATCHDOG TARGET: SCL sampled high after release' 'WATCHDOG TARGET: SCL sampled high after release'
    if ($TargetLog -match 'WATCHDOG TARGET: SCL observer unavailable:') {
        throw 'SCL observer unavailable contradicts the required electrical samples.'
    }
    $recoveryObservations = Get-WatchdogRecovery $Mode $ControllerLog $Recovery
    $repair = Get-WatchdogRepair $Recovery $ControllerLog $TargetLog
    return [ordered]@{
        configuration = [ordered]@{
            requested_ms = $readback.requested; input_hz = $readback.hz; prescale = $readback.prescale
            pinlow = $readback.pinlow; timecfg = $readback.timecfg; filtscl = $readback.filter
            software_timeout_ms = $readback.timeout
            nominal_threshold_us = ($readback.pinlow * 256.0 * 1000000 / $readback.hz)
        }
        controller = [ordered]@{
            result = $result; elapsed_us = $controllerUs; accepted_elapsed_us = $window
            acceptance_scope = 'Board-selected tolerance window, not a precision bound.'
        }
        recovery = $recoveryObservations
        repair = $repair
        target = [ordered]@{
            unserviced_rdf_samples = $samples; hold_elapsed_us = $holdUs
            accepted_hold_us = @(150000, 175000); rdde_erq_active_off_at_samples = $true
        }
        scl = [ordered]@{
            sampled = $true; sample_evidence = 'discrete_input_samples'
            threshold_witness = ($Mode -ne '200'); completing_upper_threshold_control = ($Mode -eq '200')
            continuous_waveform_qualified = $false
            scope = 'Hardware-qualified SCL-only threshold, subject to input sampling, filtering and clock tolerance; separate runs do not bracket one event.'
        }
    }
}

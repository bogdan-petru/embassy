# Hardware-free controls; no runner, flashing or firmware build is loaded.
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/../watchdog-verdicts.ps1"
$passed = 0
function Check([bool]$Condition, [string]$Name) {
    if (-not $Condition) { throw "FAILED: $Name" }
    $script:passed++
}
function Reject([scriptblock]$Action, [string]$Name) {
    $rejected = $false
    try { & $Action } catch { $rejected = $true }
    Check $rejected $Name
}
function Fixture([string]$Mode) {
    $pinlow = @{ '120' = 1407; '140' = 1641; '200' = 2344 }[$Mode]
    $elapsed = @{ '120' = 120500; '140' = 140500; '200' = 151000 }[$Mode]
    $result = if ($Mode -eq '200') { 'Ok(())' } else { 'Err(PinLowTimeout)' }
    $msr = 0
    $returnScl = if ($Mode -eq '200') { 'true' } else { 'false' }
    $markers = Get-WatchdogMarkers $Mode
    return @{
        controller = "0.0 [INFO ] WATCHDOG CONTROLLER: 20s quiet window`n" +
            "0.0 [INFO ] WATCHDOG CONFIG: requested_ms=$Mode, input_hz=3000000, prescale=0, pinlow=$pinlow, timecfg=0, filtscl=0, software_timeout_ms=500 (ctrl check.rs:20)`n" +
            "10.0 [INFO ] WATCHDOG CONTROLLER result=$result, elapsed_us=$elapsed (ctrl check.rs:40)`n" +
            "10.0 [INFO ] WATCHDOG RECOVERY: stage=return, mcr=1, msr=$msr, mfsr=0, scl_high=$returnScl`n" +
            "10.1 [INFO ] WATCHDOG RECOVERY: stage=after_wait, mcr=1, msr=$msr, mfsr=0, scl_high=true`n" +
            "10.1 [INFO ] $($markers.controller)`nFirmware exited successfully`n"
        target = "0.0 [INFO ] WATCHDOG TARGET READY: $Mode`n" +
            "0.0 [INFO ] WATCHDOG TARGET: SCL sampled idle-high`n" +
            "6.0 [INFO ] WATCHDOG TARGET: unserviced-RDF samples=149, elapsed_us=150273, RDDE/ERQ/ACTIVE off, SCL sampled=true (target check.rs:100)`n" +
            "6.1 [INFO ] WATCHDOG TARGET: SCL sampled high after release`n" +
            "6.2 [INFO ] $($markers.target)`nFirmware exited successfully`n"
    }
}
foreach ($mode in @('120', '140', '200')) {
    $fixture = Fixture $mode
    $markers = Get-WatchdogMarkers $mode
    foreach ($role in @('controller', 'target')) {
        $log = $fixture[$role]
        Assert-WatchdogEndpoint $mode $role 0 $log
        Check $true "$mode/$role exact marker plus exit zero"
        Reject { Assert-WatchdogEndpoint $mode $role 1 $log } "$mode/$role nonzero exit"
        Reject { Assert-WatchdogEndpoint $mode $role 0 '' } "$mode/$role missing marker"
        Reject { Assert-WatchdogEndpoint $mode $role 0 ($log + $log) } "$mode/$role duplicate marker"
        $marker = $markers[$role]
        foreach ($bad in @("0.0 [INFO ] prefix $marker", "0.0 [INFO ] $marker but not really", "0.0 [ERROR] $marker")) {
            Reject { Assert-WatchdogEndpoint $mode $role 0 $bad } "$mode/$role malformed or wrong-level marker"
        }
        $wrongMode = if ($mode -eq '120') { '140' } else { '120' }
        Reject { Assert-WatchdogEndpoint $mode $role 0 ($log -replace "WATCHDOG $mode ", "WATCHDOG $wrongMode ") } "$mode/$role wrong mode marker"
        $other = (Get-WatchdogMarkers $wrongMode)[$role]
        Reject { Assert-WatchdogEndpoint $mode $role 0 ($log + "0.0 [INFO ] $other`n") } "$mode/$role conflicting mode marker"
    }
    $observation = Get-WatchdogObservations $mode $fixture.controller $fixture.target
    Check ($observation.scl.threshold_witness -eq ($mode -ne '200')) "$mode only faults are threshold witnesses"
    Check ($observation.scl.completing_upper_threshold_control -eq ($mode -eq '200')) "$mode upper control labelled separately"
    Check (-not $observation.scl.continuous_waveform_qualified) "$mode does not claim waveform qualification"
    $nominal = @{ '120' = 120064.0; '140' = 140032.0; '200' = (600064.0 / 3) }[$mode]
    Check ([Math]::Abs($observation.configuration.nominal_threshold_us - $nominal) -lt 0.001) "$mode nominal threshold records encoded register value"
    Check ($observation.configuration.prescale -eq 0 -and $observation.configuration.timecfg -eq 0) "$mode SCL-only no-prescale readback"
    Check (-not $observation.recovery.terminal_recovery_qualified) "$mode threshold pass does not qualify terminal recovery"
    Check $observation.recovery.local_engine_recovery_qualified "$mode unrepaired snapshots qualify only local-engine recovery"
    Check $observation.recovery.controller_idle_at_return "$mode unrepaired engine must be idle at return"
    Check $observation.recovery.controller_idle_after_wait "$mode unrepaired engine must remain idle after wait"
    $recoveryLines = @($fixture.controller -split "`n" | Where-Object { $_.Contains('WATCHDOG RECOVERY:') })
    foreach ($line in $recoveryLines) {
        Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($line + "`n", '')) $fixture.target } "$mode missing recovery stage"
        Reject { Get-WatchdogObservations $mode ($fixture.controller + $line + "`n") $fixture.target } "$mode duplicate recovery stage"
        foreach ($field in @('mcr', 'msr', 'mfsr')) {
            foreach ($bad in @('4294967296', '-1', 'oops')) {
                $changed = $line -replace ($field + '=\d+'), ($field + '=' + $bad)
                Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($line, $changed)) $fixture.target } "$mode recovery $field wrong width/malformed $bad"
            }
        }
        foreach ($replacement in @('mcr=0', 'mcr=2', 'mfsr=1', 'mfsr=65536')) {
            $field = $replacement.Split('=')[0]
            $changed = $line -replace ($field + '=\d+'), $replacement
            Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($line, $changed)) $fixture.target } "$mode recovery disabled MEN/nonempty FIFO $replacement"
        }
        foreach ($bad in @('0', 'oops')) {
            $changed = $line -replace 'scl_high=(true|false)', "scl_high=$bad"
            Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($line, $changed)) $fixture.target } "$mode recovery malformed SCL $bad"
        }
        $changed = $line -replace 'msr=\d+', 'msr=16777216'
        Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($line, $changed)) $fixture.target } "$mode default must be controller-idle at both snapshots"
    }
    Reject { Get-WatchdogObservations $mode ($fixture.controller -replace 'stage=return,', 'stage=wrong,') $fixture.target } "$mode unknown recovery stage rejected"
    Reject { Get-WatchdogObservations $mode ($fixture.controller + "10.2 [INFO ] WATCHDOG RECOVERY: stage=extra, mcr=1, msr=0, mfsr=0, scl_high=true`n") $fixture.target } "$mode extra recovery stage rejected"
    $reversed = $fixture.controller.Replace($recoveryLines[0], '__FIRST__').Replace($recoveryLines[1], $recoveryLines[0]).Replace('__FIRST__', $recoveryLines[1])
    Reject { Get-WatchdogObservations $mode $reversed $fixture.target } "$mode snapshots cannot be reversed"
    $badAfter = $recoveryLines[1] -replace 'scl_high=true', 'scl_high=false'
    Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($recoveryLines[1], $badAfter)) $fixture.target } "$mode after-wait SCL must be high"
    $onlyBbf = $fixture.controller -replace 'msr=\d+', 'msr=33554432'
    $bbfObservation = Get-WatchdogObservations $mode $onlyBbf $fixture.target
    Check ($bbfObservation.recovery.controller_idle_at_return -and $bbfObservation.recovery.controller_idle_after_wait) "$mode BBF alone is not controller MBF"
    Check ($bbfObservation.recovery.snapshots['return'].msr -eq 33554432) "$mode full raw status retained"
    Check ($bbfObservation.recovery.local_engine_recovery_qualified -and -not $bbfObservation.recovery.terminal_recovery_qualified) "$mode local idle with BBF set is not full bus recovery"
    foreach ($prefix in @('WATCHDOG CONFIG:', 'WATCHDOG CONTROLLER result=', 'WATCHDOG CONTROLLER: 20s quiet window')) {
        $line = @($fixture.controller -split "`n" | Where-Object { $_.Contains($prefix) })[0]
        $without = $fixture.controller.Replace($line + "`n", '')
        Reject { Get-WatchdogObservations $mode $without $fixture.target } "$mode missing $prefix"
        Reject { Get-WatchdogObservations $mode ($fixture.controller + $line + "`n") $fixture.target } "$mode duplicate $prefix"
    }
    foreach ($prefix in @('WATCHDOG TARGET: unserviced-RDF', 'WATCHDOG TARGET READY:',
            'WATCHDOG TARGET: SCL sampled idle-high', 'WATCHDOG TARGET: SCL sampled high after release')) {
        $line = @($fixture.target -split "`n" | Where-Object { $_.Contains($prefix) })[0]
        $without = $fixture.target.Replace($line + "`n", '')
        Reject { Get-WatchdogObservations $mode $fixture.controller $without } "$mode missing $prefix"
        Reject { Get-WatchdogObservations $mode $fixture.controller ($fixture.target + $line + "`n") } "$mode duplicate $prefix"
    }
    foreach ($field in @('requested_ms', 'input_hz', 'prescale', 'pinlow', 'timecfg', 'filtscl', 'software_timeout_ms')) {
        foreach ($bad in @('1', '-1', '4294967296', 'oops')) {
            $changed = $fixture.controller -replace ($field + '=\d+'), ($field + '=' + $bad)
            Reject { Get-WatchdogObservations $mode $changed $fixture.target } "$mode $field mismatch/malformed/overflow $bad"
        }
    }
    foreach ($bad in @('Err(Timeout)', 'Err(AddressNack)', 'Err(FifoError)', 'Ok', '')) {
        $changed = $fixture.controller -replace 'result=[^,]*', "result=$bad"
        Reject { Get-WatchdogObservations $mode $changed $fixture.target } "$mode wrong result $bad"
    }
    $opposite = if ($mode -eq '200') { 'Err(PinLowTimeout)' } else { 'Ok(())' }
    Reject { Get-WatchdogObservations $mode ($fixture.controller -replace 'result=[^,]*', "result=$opposite") $fixture.target } "$mode fault/control reversal"
    $window = @{ '120' = @(110000, 135000); '140' = @(130000, 155000); '200' = @(145000, 190000) }[$mode]
    foreach ($edge in $window) {
        $accepted = Get-WatchdogObservations $mode ($fixture.controller -replace 'elapsed_us=\d+', "elapsed_us=$edge") $fixture.target
        Check ($accepted.controller.elapsed_us -eq $edge) "$mode inclusive controller boundary $edge"
    }
    foreach ($bad in @(($window[0] - 1), ($window[1] + 1), '-1', '18446744073709551616', 'oops')) {
        $changed = $fixture.controller -replace 'elapsed_us=\d+', "elapsed_us=$bad"
        Reject { Get-WatchdogObservations $mode $changed $fixture.target } "$mode controller time malformed/outside/overflow $bad"
    }
    foreach ($edge in @(150000, 175000)) {
        $accepted = Get-WatchdogObservations $mode $fixture.controller ($fixture.target -replace 'elapsed_us=\d+', "elapsed_us=$edge")
        Check ($accepted.target.hold_elapsed_us -eq $edge) "$mode inclusive hold boundary $edge"
    }
    foreach ($bad in @('149999', '175001', '-1', '18446744073709551616', 'oops')) {
        Reject { Get-WatchdogObservations $mode $fixture.controller ($fixture.target -replace 'elapsed_us=\d+', "elapsed_us=$bad") } "$mode hold duration malformed/outside/overflow $bad"
    }
    foreach ($bad in @('0', '1', '-1', '4294967296', 'oops')) {
        Reject { Get-WatchdogObservations $mode $fixture.controller ($fixture.target -replace 'samples=149', "samples=$bad") } "$mode samples malformed/outside/overflow $bad"
    }
    $two = Get-WatchdogObservations $mode $fixture.controller ($fixture.target -replace 'samples=149', 'samples=2')
    Check ($two.target.unserviced_rdf_samples -eq 2) "$mode requires measured hold not idealized timer cadence"
    Reject { Get-WatchdogObservations $mode $fixture.controller ($fixture.target -replace 'RDDE/ERQ/ACTIVE off', 'RDDE/ERQ/ACTIVE unknown') } "$mode disabled RX service proof required"
    foreach ($bad in @('false', '0', 'oops')) {
        Reject { Get-WatchdogObservations $mode $fixture.controller ($fixture.target -replace 'SCL sampled=true', "SCL sampled=$bad") } "$mode sampled SCL mandatory $bad"
    }
    Reject { Get-WatchdogObservations $mode $fixture.controller ($fixture.target + "0.0 [WARN ] WATCHDOG TARGET: SCL observer unavailable: test`n") } "$mode no unavailable observer warning"
    Reject { Get-WatchdogObservations $mode $fixture.controller ($fixture.target -replace "WATCHDOG TARGET READY: $mode", 'WATCHDOG TARGET READY: 999') } "$mode wrong READY mode"
    $colored = ([char]27 + '[32m') + $fixture.controller + ([char]27 + '[0m')
    $null = Get-WatchdogObservations $mode $colored $fixture.target
    Check $true "$mode ANSI log wrapper accepted"
}
function RepairFixture([string]$Mode, [string]$Recovery) {
    $fixture = Fixture $Mode
    # Historical intervention diagnostics begin with the original busy-engine
    # failure. They must remain replayable without qualifying production code.
    $returned = @($fixture.controller -split "`n" | Where-Object { $_.Contains('stage=return,') })[0]
    $fixture.controller = $fixture.controller.Replace($returned, ($returned -replace 'msr=\d+', 'msr=16777216'))
    $waitField = if ($Recovery -in @('abrupt', 'abrupt_idle')) { 'reset_wait_us' } else { 'disable_wait_us' }
    $checkpoint = if ($Recovery -in @('abrupt', 'abrupt_idle')) {
        'stage=reset_complete, mcr=1, msr=0, mfsr=0, scl_high=false'
    } else {
        'stage=disabled, mcr=0, msr=0, mfsr=0, scl_high=true'
    }
    $afterWait = @($fixture.controller -split "`n" | Where-Object { $_.Contains('stage=after_wait,') })[0]
    $fixture.controller = $fixture.controller.Replace($afterWait,
        "10.0 [INFO ] WATCHDOG REPAIR: mode=$Recovery`n" +
        "10.0 [INFO ] WATCHDOG REPAIR: $waitField=5`n" +
        "10.0 [INFO ] WATCHDOG REPAIR: configuration restored=true`n" +
        "10.1 [INFO ] WATCHDOG REPAIR: $checkpoint`n" +
        ($afterWait -replace 'msr=\d+', 'msr=0'))
    $marker = (Get-WatchdogMarkers $Mode).controller
    $fixture.controller = $fixture.controller.Replace("10.1 [INFO ] $marker",
        "10.2 [INFO ] WATCHDOG REPAIR: fresh write PASS`n10.2 [INFO ] $marker")
    if ($Recovery -eq 'abrupt_idle') {
        $fixture.controller = $fixture.controller.Replace('10.0 [INFO ] WATCHDOG REPAIR: configuration restored=true',
            "10.0 [INFO ] WATCHDOG REPAIR: idle_detector busidle=52, clklo=20, sethold=4, clkhi=14`n10.0 [INFO ] WATCHDOG REPAIR: configuration restored=true")
        $fixture.controller = $fixture.controller.Replace('10.2 [INFO ] WATCHDOG REPAIR: fresh write PASS',
            "10.2 [INFO ] WATCHDOG REPAIR: lines scl_high=true, sda_high=true, bus_busy=false`n10.2 [INFO ] WATCHDOG REPAIR: fresh write PASS")
    }
    $ready = "0.0 [INFO ] WATCHDOG TARGET READY: $Mode`n"
    $fixture.target = $fixture.target.Replace($ready, $ready + "0.0 [INFO ] WATCHDOG REPAIR: mode=$Recovery`n")
    $marker = (Get-WatchdogMarkers $Mode).target
    $fixture.target = $fixture.target.Replace("6.2 [INFO ] $marker",
        "6.2 [INFO ] WATCHDOG REPAIR: fresh exact 16 bytes and STOP`n6.2 [INFO ] $marker")
    return $fixture
}
foreach ($mode in @('120', '140')) {
    foreach ($repair in @('wait', 'reset', 'abrupt', 'abrupt_idle')) {
        $fixture = RepairFixture $mode $repair
        $abrupt = $repair -in @('abrupt', 'abrupt_idle')
        $waitField = if ($abrupt) { 'reset_wait_us' } else { 'disable_wait_us' }
        $waitLimit = if ($abrupt) { 1000 } else { 300000 }
        $checkpointField = if ($abrupt) { 'reset_complete_checkpoint' } else { 'disabled_checkpoint' }
        $observation = Get-WatchdogObservations $mode $fixture.controller $fixture.target $repair
        Check ($observation.repair.mode -eq $repair -and $observation.repair.diagnostic_repair_qualified) "$mode/$repair selected repair qualified"
        Check (-not $observation.recovery.terminal_recovery_qualified) "$mode/$repair does not qualify unchanged production recovery"
        Check (-not $observation.recovery.local_engine_recovery_qualified) "$mode/$repair external intervention does not qualify production local-engine recovery"
        Check (-not $observation.recovery.controller_idle_at_return -and $observation.recovery.controller_idle_after_wait) "$mode/$repair original failure retained then repaired idle required"
        Check ($observation.repair[$waitField] -eq 5 -and $observation.repair.fresh_exact_transfer_and_stop) "$mode/$repair acknowledgment and fresh transfer recorded"
        Check ($observation.repair[$checkpointField].scl_high -eq (-not $abrupt)) "$mode/$repair requires the mode-specific SCL level at the checkpoint"
        $otherWaitField = if ($abrupt) { 'disable_wait_us' } else { 'reset_wait_us' }
        $otherStage = if ($abrupt) { 'disabled' } else { 'reset_complete' }
        Check (-not $observation.repair.Contains($otherWaitField)) "$mode/$repair reset acknowledgment is not mislabeled as graceful disable latency"
        Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($waitField, $otherWaitField)) $fixture.target $repair } "$mode/$repair wrong acknowledgment kind rejected"
        Reject { Get-WatchdogObservations $mode ($fixture.controller -replace 'WATCHDOG REPAIR: stage=\w+,', "WATCHDOG REPAIR: stage=$otherStage,") $fixture.target $repair } "$mode/$repair wrong checkpoint stage rejected"
        Reject { Get-WatchdogObservations $mode $fixture.controller $fixture.target } "$mode/$repair records forbidden when runner selected none"
        $plain = Fixture $mode
        Reject { Get-WatchdogObservations $mode $plain.controller $plain.target $repair } "$mode/$repair cannot silently run unmodified firmware"
        foreach ($role in @('controller', 'target')) {
            $lines = @($fixture[$role] -split "`n" | Where-Object { $_.Contains('WATCHDOG REPAIR:') })
            foreach ($line in $lines) {
                foreach ($changed in @('', ($line + "`n" + $line), ($line -replace '\[INFO \]', '[WARN ]'),
                        ($line -replace 'WATCHDOG REPAIR:', 'prefix WATCHDOG REPAIR:'), ($line + ' extra'))) {
                    $logs = @{ controller = $fixture.controller; target = $fixture.target }
                    $logs[$role] = $logs[$role].Replace($line, $changed)
                    Reject { Get-WatchdogObservations $mode $logs.controller $logs.target $repair } "$mode/$repair/$role missing/duplicate/malformed repair record: $line"
                }
            }
            $logs = @{ controller = $fixture.controller; target = $fixture.target }
            $logs[$role] += "12.0 [INFO ] WATCHDOG REPAIR: unknown evidence=true`n"
            Reject { Get-WatchdogObservations $mode $logs.controller $logs.target $repair } "$mode/$repair/$role unexpected repair record rejected"
            $logs = @{ controller = $fixture.controller; target = $fixture.target }
            $opposite = if ($repair -eq 'wait') { 'reset' } else { 'wait' }
            $logs[$role] = $logs[$role] -replace "WATCHDOG REPAIR: mode=$repair", "WATCHDOG REPAIR: mode=$opposite"
            Reject { Get-WatchdogObservations $mode $logs.controller $logs.target $repair } "$mode/$repair/$role mode must match runner"
        }
        foreach ($boundary in @(0, $waitLimit)) {
            $changed = $fixture.controller -replace "$waitField=5", "$waitField=$boundary"
            $observed = Get-WatchdogObservations $mode $changed $fixture.target $repair
            Check ($observed.repair[$waitField] -eq $boundary) "$mode/$repair inclusive acknowledgment bound $boundary"
        }
        foreach ($bad in @(($waitLimit + 1), '-1', '18446744073709551616', 'oops')) {
            $changed = $fixture.controller -replace "$waitField=5", "$waitField=$bad"
            Reject { Get-WatchdogObservations $mode $changed $fixture.target $repair } "$mode/$repair acknowledgment bound malformed/outside/overflow $bad"
        }
        foreach ($bad in @('false', '1', 'oops')) {
            $changed = $fixture.controller -replace 'configuration restored=true', "configuration restored=$bad"
            Reject { Get-WatchdogObservations $mode $changed $fixture.target $repair } "$mode/$repair restored configuration assertion required $bad"
        }
        $checkpoint = @($fixture.controller -split "`n" | Where-Object { $_.Contains('WATCHDOG REPAIR: stage=') })[0]
        $badMen = if ($abrupt) { 'mcr=0' } else { 'mcr=1' }
        $badScl = if ($abrupt) { 'scl_high=true' } else { 'scl_high=false' }
        foreach ($replacement in @($badMen, 'mcr=2', 'mcr=3', 'msr=16777216', 'mfsr=1', 'mfsr=65536', $badScl)) {
            $field = $replacement.Split('=')[0]
            $changed = $checkpoint -replace ($field + '=\w+'), $replacement
            Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($checkpoint, $changed)) $fixture.target $repair } "$mode/$repair checkpoint invariant $replacement"
        }
        foreach ($field in @('mcr', 'msr', 'mfsr')) {
            foreach ($bad in @('4294967296', '-1', 'oops')) {
                $changed = $checkpoint -replace ($field + '=\d+'), ($field + '=' + $bad)
                Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($checkpoint, $changed)) $fixture.target $repair } "$mode/$repair checkpoint $field malformed/overflow $bad"
            }
        }
        $bbf = $checkpoint -replace 'msr=0', 'msr=33554432'
        $observed = Get-WatchdogObservations $mode ($fixture.controller.Replace($checkpoint, $bbf)) $fixture.target $repair
        Check ($observed.repair[$checkpointField].controller_idle) "$mode/$repair checkpoint BBF is not MBF"
        $afterWait = @($fixture.controller -split "`n" | Where-Object { $_.Contains('stage=after_wait,') })[0]
        $busyAfter = $afterWait -replace 'msr=0', 'msr=16777216'
        Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($afterWait, $busyAfter)) $fixture.target $repair } "$mode/$repair after-wait busy cannot pass a repair"
        if ($repair -eq 'abrupt_idle') {
            Check (-not $observation.repair.configuration_restored_or_unchanged) "$mode/$repair does not claim the changed BUSIDLE was merely restored"
            Check ($observation.repair.configuration_matches_declared_repair -and $null -ne $observation.repair.diagnostic_configuration_change) "$mode/$repair declares the diagnostic configuration change"
            Check ($observation.repair.bus_idle_configuration.busidle -eq 52 -and $observation.repair.both_lines_high_and_bus_not_busy_after_wait) "$mode/$repair records the idle detector and both-line evidence"
            $bbfAfter = $afterWait -replace 'msr=0', 'msr=33554432'
            Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($afterWait, $bbfAfter)) $fixture.target $repair } "$mode/$repair BBF after wait contradicts idle detector qualification"
            foreach ($field in @('busidle', 'clklo', 'sethold', 'clkhi')) {
                $widthOverflow = if ($field -eq 'busidle') { '4096' } else { '64' }
                foreach ($bad in @($widthOverflow, '-1', '4294967296', 'oops')) {
                    $changed = $fixture.controller -replace ($field + '=\d+'), ($field + '=' + $bad)
                    Reject { Get-WatchdogObservations $mode $changed $fixture.target $repair } "$mode/$repair $field malformed/width-overflow $bad"
                }
            }
            foreach ($bad in @('0', '51', '53', '4095')) {
                $changed = $fixture.controller -replace 'busidle=52', "busidle=$bad"
                Reject { Get-WatchdogObservations $mode $changed $fixture.target $repair } "$mode/$repair BUSIDLE exact timing formula required $bad"
            }
            foreach ($timing in @('busidle=4, clklo=0, sethold=0, clkhi=3', 'busidle=4, clklo=0, sethold=0, clkhi=63')) {
                $changed = $fixture.controller.Replace('busidle=52, clklo=20, sethold=4, clkhi=14', $timing)
                Reject { Get-WatchdogObservations $mode $changed $fixture.target $repair } "$mode/$repair BUSIDLE must exceed CLKHI+1"
            }
            foreach ($timing in @('busidle=4, clklo=0, sethold=0, clkhi=0', 'busidle=256, clklo=63, sethold=63, clkhi=63')) {
                $changed = $fixture.controller.Replace('busidle=52, clklo=20, sethold=4, clkhi=14', $timing)
                $null = Get-WatchdogObservations $mode $changed $fixture.target $repair
                Check $true "$mode/$repair valid timing width endpoint $timing"
            }
            $lineEvidence = @($fixture.controller -split "`n" | Where-Object { $_.Contains('WATCHDOG REPAIR: lines ') })[0]
            foreach ($field in @('scl_high', 'sda_high', 'bus_busy')) {
                $opposite = if ($field -eq 'bus_busy') { 'true' } else { 'false' }
                foreach ($bad in @($opposite, '1', 'oops')) {
                    $changed = $lineEvidence -replace ($field + '=\w+'), ($field + '=' + $bad)
                    Reject { Get-WatchdogObservations $mode ($fixture.controller.Replace($lineEvidence, $changed)) $fixture.target $repair } "$mode/$repair line/busy evidence $field=$bad cannot pass"
                }
            }
        } else {
            Check ($observation.repair.configuration_restored_or_unchanged -and $null -eq $observation.repair.diagnostic_configuration_change) "$mode/$repair does not acquire the idle-detector configuration change"
            Check ($null -eq $observation.repair.bus_idle_configuration -and $null -eq $observation.repair.both_lines_high_and_bus_not_busy_after_wait) "$mode/$repair does not claim unobserved SDA or idle-detector evidence"
            foreach ($extra in @('idle_detector busidle=52, clklo=20, sethold=4, clkhi=14', 'lines scl_high=true, sda_high=true, bus_busy=false')) {
                Reject { Get-WatchdogObservations $mode ($fixture.controller + "12.0 [INFO ] WATCHDOG REPAIR: $extra`n") $fixture.target $repair } "$mode/$repair cannot silently include abrupt_idle evidence"
            }
        }
        foreach ($role in @('controller', 'target')) {
            $sequence = @($fixture[$role] -split "`n" | Where-Object {
                    $_.Contains('WATCHDOG REPAIR:') -or $_.Contains('WATCHDOG RECOVERY:') -or
                    $_.Contains('WATCHDOG TARGET READY:') -or $_.Contains('WATCHDOG TARGET: unserviced-RDF') -or
                    $_.Contains('WATCHDOG TARGET: SCL sampled high after release')
                })
            for ($index = 0; $index -lt $sequence.Count - 1; $index++) {
                $logs = @{ controller = $fixture.controller; target = $fixture.target }
                $logs[$role] = $logs[$role].Replace($sequence[$index], '__FIRST__').Replace($sequence[$index + 1], $sequence[$index]).Replace('__FIRST__', $sequence[$index + 1])
                Reject { Get-WatchdogObservations $mode $logs.controller $logs.target $repair } "$mode/$repair/$role reordered evidence $index"
            }
        }
    }
    $fixture = Fixture $mode
    $observed = Get-WatchdogObservations $mode $fixture.controller $fixture.target
    Check ($observed.repair.mode -eq 'none' -and -not $observed.repair.diagnostic_repair_qualified) "$mode default stays unrepaired"
    foreach ($role in @('controller', 'target')) {
        $logs = @{ controller = $fixture.controller; target = $fixture.target }
        $logs[$role] += "12.0 [WARN ] WATCHDOG REPAIR: unknown evidence=true`n"
        Reject { Get-WatchdogObservations $mode $logs.controller $logs.target } "$mode/$role default rejects even malformed repair evidence"
    }
}
foreach ($repair in @('wait', 'reset', 'abrupt', 'abrupt_idle')) {
    Assert-WatchdogModes @('120', '140') $repair
    Check $true "$repair fault-only selection accepted"
    Reject { Assert-WatchdogModes @('200') $repair } "$repair cannot repair control case"
    Reject { Assert-WatchdogModes @('120', '140', '200') $repair } "$repair cannot silently include control in sweep"
    $fixture = Fixture '200'
    Reject { Get-WatchdogObservations '200' $fixture.controller $fixture.target $repair } "$repair observation rejects control mode"
}
Reject { Assert-WatchdogModes @('120') 'unknown' } 'unknown recovery selection rejected'
Reject { Get-WatchdogObservations '120' '' '' 'unknown' } 'unknown recovery observation rejected'
Assert-RxstallStartup $true 18999 19000
Check $true 'READY inside conservative budget'
Reject { Assert-RxstallStartup $true 19000 19000 } 'READY at boundary rejected'
Reject { Assert-RxstallStartup $false 100 19000 } 'missing READY rejected'
Reject { Get-WatchdogMarkers '160' } 'unsupported former control mode rejected'
Reject { Get-WatchdogObservations '160' '' '' } 'unsupported observation mode rejected'
foreach ($selection in @(@('120', '140', '200'), @('140', '200'), @('200'), @('200', '140', '120'))) {
    Assert-WatchdogModes $selection
    Check $true "distinct selection accepted: $($selection -join ',')"
}
Reject { Assert-WatchdogModes @() } 'empty mode selection rejected'
Reject { Assert-WatchdogModes $null } 'null mode selection rejected'
Reject { Assert-WatchdogModes @('120', '120') } 'duplicate case cannot become an implicit retry'
Reject { Assert-WatchdogModes @('140', '200', '140') } 'nonadjacent duplicate mode rejected'
Reject { Assert-WatchdogModes @('160') } 'unknown selected mode rejected'
foreach ($results in @(@('fail', 'pass'), @('pass', 'fail'), @('fail', 'pass', 'pass'), @('pass', 'fail', 'pass'))) {
    Check ((Get-WatchdogAggregateResult $results) -eq 'fail') "later/earlier PASS cannot erase FAIL: $($results -join ',')"
}
Check ((Get-WatchdogAggregateResult @('pass', 'pass', 'pass')) -eq 'pass') 'all requested cases must pass for aggregate pass'
Check ((Get-WatchdogAggregateResult @('pass', 'pass')) -eq 'pass') 'a subset can pass without being labelled full sweep'
Reject { Get-WatchdogAggregateResult @() } 'empty results cannot pass'
Reject { Get-WatchdogAggregateResult $null } 'missing results cannot pass'
Reject { Get-WatchdogAggregateResult @('pass', 'running') } 'unfinished result cannot pass'
Reject { Get-WatchdogAggregateResult @('fail', 'unknown') } 'unknown result not hidden by another failure'
Write-Output "WATCHDOG verdict controls: $passed passed"

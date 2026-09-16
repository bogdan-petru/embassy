# Pure verdict rules shared by the board runner and its host-side controls.
# A successful process alone, a quoted/substr matching message, or a verdict
# from the other mode must never be accepted as this mode's firmware verdict.
function Test-RxstallMarker([string]$Log, [string]$Marker) {
    $plain = $Log -replace ([char]27 + '\[[0-9;]*m'), ''
    $pattern = '(?m)^\s*(?:\d+(?:\.\d+)?\s+)?\[INFO\s*\]\s+' +
        [regex]::Escape($Marker) + '(?: \([^()\r\n]+\))?\s*\r?$'
    return [regex]::IsMatch($plain, $pattern)
}

function Get-RxstallMarkers([ValidateSet('short', 'long')][string]$Mode) {
    if ($Mode -eq 'short') {
        return @{
            controller = 'RXSTALL SHORT CONTROLLER PASS'
            target = 'RXSTALL SHORT TARGET PASS: exact 16 bytes and STOP'
        }
    }
    return @{
        controller = 'RXSTALL LONG CONTROLLER PASS: exact Timeout and local cleanup'
        target = 'RXSTALL LONG TARGET PASS: withheld RX service and explicit target abort'
    }
}

function Assert-RxstallEndpoint([string]$Role, [int]$ExitCode, [string]$Log, [string]$Marker) {
    if ($ExitCode -ne 0) { throw "$Role failed: exit $ExitCode" }
    if (-not (Test-RxstallMarker $Log $Marker)) { throw "$Role missing exact verdict: $Marker" }
}

function Assert-RxstallStartup([bool]$Ready, [long]$ElapsedMs, [long]$BudgetMs = 9000) {
    if (-not $Ready) { throw 'Target did not publish its mode-specific READY marker.' }
    if ($ElapsedMs -ge $BudgetMs) {
        throw "Target READY was too late: ${ElapsedMs}ms, must be below ${BudgetMs}ms from controller launch."
    }
}

function Get-RxstallRecord([string]$Log, [string]$Level, [string]$Prefix, [string]$BodyPattern) {
    $plain = $Log -replace ([char]27 + '\[[0-9;]*m'), ''
    $start = '(?m)^[ \t]*(?:\d+(?:\.\d+)?[ \t]+)?\[' + [regex]::Escape($Level) + '[ \t]*\][ \t]+'
    $candidates = [regex]::Matches($plain, $start + [regex]::Escape($Prefix) + '[^\r\n]*')
    if ($candidates.Count -ne 1) { throw "Expected exactly one $Level record beginning '$Prefix'; found $($candidates.Count)." }
    $records = [regex]::Matches($plain, $start + $BodyPattern + '(?: \([^()\r\n]+\))?[ \t]*\r?$')
    if ($records.Count -ne 1) { throw "Malformed $Level telemetry beginning '$Prefix'." }
    return $records[0]
}

function Get-RxstallObservations([ValidateSet('short', 'long')][string]$Mode, [string]$ControllerLog, [string]$TargetLog) {
    $controller = Get-RxstallRecord $ControllerLog 'INFO' 'RXSTALL CONTROLLER result=' `
        'RXSTALL CONTROLLER result=(?<result>[^,\r\n]+), elapsed_us=(?<elapsed>[0-9]+)'
    $target = Get-RxstallRecord $TargetLog 'INFO' 'RXSTALL TARGET: unserviced-RDF' `
        'RXSTALL TARGET: unserviced-RDF samples=(?<samples>[0-9]+), elapsed_us=(?<elapsed>[0-9]+), RDDE/ERQ/ACTIVE off, SCL sampled=(?<sampled>true|false)'
    # Parse explicitly into the firmware counter widths; overflow must fail,
    # not turn a malformed log into a missing or rounded measurement.
    $controllerUs = [uint64]::Parse($controller.Groups['elapsed'].Value)
    $holdUs = [uint64]::Parse($target.Groups['elapsed'].Value)
    $samples = [uint32]::Parse($target.Groups['samples'].Value)
    $result = $controller.Groups['result'].Value
    $sampled = $target.Groups['sampled'].Value -eq 'true'
    if ($Mode -eq 'short') {
        $expectedResult = 'Ok(())'
        $minimumControllerUs = 5000
        $maximumControllerUs = 50000
        $minimumHoldUs = 5000
    } else {
        $expectedResult = 'Err(Timeout)'
        $minimumControllerUs = 95000
        $maximumControllerUs = 125000
        $minimumHoldUs = 150000
    }
    if ($result -ne $expectedResult) { throw "$Mode controller result '$result', expected '$expectedResult'." }
    if ($controllerUs -lt $minimumControllerUs -or $controllerUs -gt $maximumControllerUs) {
        throw "$Mode controller elapsed_us=$controllerUs outside $minimumControllerUs..$maximumControllerUs."
    }
    # Initial + at least one later sample. Requiring 6/151 samples would assume
    # perfectly punctual 1 ms timer wakes rather than checking measured time.
    if ($samples -lt 2) { throw "$Mode needs at least two unserviced-RDF samples." }
    if ($holdUs -lt $minimumHoldUs) { throw "$Mode RX hold elapsed_us=$holdUs below $minimumHoldUs." }
    if ($Mode -eq 'short' -and $holdUs -ge 100000) { throw 'Short RX hold crossed the 100 ms controller deadline.' }

    $idleHigh = Test-RxstallMarker $TargetLog 'RXSTALL TARGET: configured SCL input sampled idle-high'
    $releasedHigh = Test-RxstallMarker $TargetLog 'RXSTALL TARGET: SCL sampled high after release; not a continuous waveform'
    $unavailable = [regex]::IsMatch($TargetLog, 'RXSTALL TARGET: SCL observer unavailable:')
    $unavailableReason = $null
    if ($sampled) {
        if (-not $idleHigh -or -not $releasedHigh -or $unavailable) {
            throw 'SCL sampled=true requires idle/release high markers and no unavailable warning.'
        }
        $electricalEvidence = 'discrete_input_samples'
    } else {
        if ($idleHigh -or $releasedHigh) { throw 'SCL sampled=false contradicts an electrical sample marker.' }
        $warning = Get-RxstallRecord $TargetLog 'WARN' 'RXSTALL TARGET: SCL observer unavailable:' `
            'RXSTALL TARGET: SCL observer unavailable: (?<reason>[^;\r\n]+); no electrical verdict'
        $unavailableReason = $warning.Groups['reason'].Value
        $electricalEvidence = 'unavailable'
    }
    return [ordered]@{
        controller = [ordered]@{
            result = $result; elapsed_us = $controllerUs
            accepted_elapsed_us = @($minimumControllerUs, $maximumControllerUs)
        }
        target = [ordered]@{
            unserviced_rdf_samples = $samples; hold_elapsed_us = $holdUs
            minimum_hold_us = $minimumHoldUs; rdde_erq_active_off_at_samples = $true
        }
        scl = [ordered]@{
            sampled = $sampled; evidence = $electricalEvidence; unavailable_reason = $unavailableReason
            continuous_waveform_qualified = $false
        }
    }
}

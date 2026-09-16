# Hardware-free checks. Only the pure verdict helpers are loaded.
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/../rxstall-verdicts.ps1"
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
foreach ($mode in @('short', 'long')) {
    $markers = Get-RxstallMarkers $mode
    foreach ($role in @('controller', 'target')) {
        $marker = $markers[$role]
        $log = "10.100012 [INFO ] $marker (rxstall src/check.rs:42)`r`nFirmware exited successfully`r`n"
        Check (Test-RxstallMarker $log $marker) "$mode/$role exact firmware marker"
        Assert-RxstallEndpoint $role 0 $log $marker
        Check $true "$mode/$role zero exit plus exact marker"
        Reject { Assert-RxstallEndpoint $role 1 $log $marker } "$mode/$role nonzero exit despite marker"
        Reject { Assert-RxstallEndpoint $role 0 '' $marker } "$mode/$role zero exit without marker"
        Check (-not (Test-RxstallMarker "0.0 [INFO ] prefix $marker" $marker)) "$mode/$role embedded marker rejected"
        Check (-not (Test-RxstallMarker "0.0 [INFO ] $marker but not really" $marker)) "$mode/$role extended marker rejected"
        Check (-not (Test-RxstallMarker "0.0 [ERROR] $marker" $marker)) "$mode/$role error-level marker rejected"
    }
}
$short = Get-RxstallMarkers 'short'
$long = Get-RxstallMarkers 'long'
Reject { Assert-RxstallEndpoint 'controller' 0 "0.0 [INFO ] $($short.controller)" $long.controller } 'short verdict cannot pass long mode'
Assert-RxstallStartup $true 8999
Check $true 'READY strictly inside conservative window'
Reject { Assert-RxstallStartup $true 9000 } 'READY exactly at deadline rejected'
Reject { Assert-RxstallStartup $true 10000 } 'READY after deadline rejected'
Reject { Assert-RxstallStartup $false 100 } 'absent READY rejected even early'

function Fixture([string]$Mode) {
    if ($Mode -eq 'short') { $result = 'Ok(())'; $controllerUs = 6880; $holdUs = 5128 } else {
        $result = 'Err(Timeout)'; $controllerUs = 100062; $holdUs = 150273
    }
    return @{
        controller = "10.0 [INFO ] RXSTALL CONTROLLER result=$result, elapsed_us=$controllerUs (ctrl check.rs:40)`n"
        target = "0.0 [WARN ] RXSTALL TARGET: SCL observer unavailable: GPIO3 clock disabled; no electrical verdict (target check.rs:88)`n" +
            "6.0 [INFO ] RXSTALL TARGET: unserviced-RDF samples=2, elapsed_us=$holdUs, RDDE/ERQ/ACTIVE off, SCL sampled=false (target check.rs:125)`n"
    }
}
foreach ($mode in @('short', 'long')) {
    $fixture = Fixture $mode
    $observation = Get-RxstallObservations $mode $fixture.controller $fixture.target
    Check ($observation.target.unserviced_rdf_samples -eq 2) "$mode two samples and sufficient measured duration accepted"
    Check (-not $observation.scl.sampled -and $observation.scl.evidence -eq 'unavailable') "$mode unavailable SCL is not electrical success"
    Check ($observation.scl.unavailable_reason -eq 'GPIO3 clock disabled') "$mode unavailable reason retained"
    Check (-not $observation.scl.continuous_waveform_qualified) "$mode no continuous-waveform claim"
    Reject { Get-RxstallObservations $mode '' $fixture.target } "$mode controller telemetry required"
    Reject { Get-RxstallObservations $mode $fixture.controller '' } "$mode target telemetry required"
    Reject { Get-RxstallObservations $mode ($fixture.controller + $fixture.controller) $fixture.target } "$mode duplicate controller telemetry rejected"
    Reject { Get-RxstallObservations $mode $fixture.controller ($fixture.target + $fixture.target) } "$mode duplicate target telemetry rejected"
    Reject { Get-RxstallObservations $mode ($fixture.controller -replace 'result=[^,]+', 'result=Err(Nack)') $fixture.target } "$mode wrong controller result rejected"
    Reject { Get-RxstallObservations $mode ($fixture.controller -replace 'elapsed_us=\d+', 'elapsed_us=4999') $fixture.target } "$mode controller duration too short"
    Reject { Get-RxstallObservations $mode ($fixture.controller -replace 'elapsed_us=\d+', 'elapsed_us=125001') $fixture.target } "$mode controller duration too long"
    Reject { Get-RxstallObservations $mode $fixture.controller ($fixture.target -replace 'elapsed_us=\d+', 'elapsed_us=4999') } "$mode hold duration too short"
    Reject { Get-RxstallObservations $mode $fixture.controller ($fixture.target -replace 'samples=2', 'samples=1') } "$mode initial sample alone rejected"
    Reject { Get-RxstallObservations $mode $fixture.controller ($fixture.target -replace 'samples=2', 'samples=4294967296') } "$mode sample count overflow rejected"
    Reject { Get-RxstallObservations $mode ($fixture.controller -replace 'elapsed_us=\d+', 'elapsed_us=18446744073709551616') $fixture.target } "$mode elapsed counter overflow rejected"
    Reject { Get-RxstallObservations $mode $fixture.controller ($fixture.target -replace 'RDDE/ERQ/ACTIVE off', 'RDDE/ERQ/ACTIVE unknown') } "$mode disabled-service assertion required"
    Reject { Get-RxstallObservations $mode $fixture.controller ($fixture.target -replace 'SCL sampled=false', 'SCL sampled=0') } "$mode malformed sampled flag rejected"
    Reject { Get-RxstallObservations $mode $fixture.controller ($fixture.target -replace '(?m)^.*\[WARN.*\r?\n', '') } "$mode unavailable warning required for false flag"
    Reject { Get-RxstallObservations $mode $fixture.controller ($fixture.target + "0.0 [INFO ] RXSTALL TARGET: configured SCL input sampled idle-high`n") } "$mode false flag contradicting high observation rejected"

    $sampled = ($fixture.target -replace '(?m)^.*\[WARN.*\r?\n', '') -replace 'SCL sampled=false', 'SCL sampled=true'
    $sampled += "0.0 [INFO ] RXSTALL TARGET: configured SCL input sampled idle-high`n"
    Reject { Get-RxstallObservations $mode $fixture.controller $sampled } "$mode true flag requires released-high observation too"
    $sampled += "6.1 [INFO ] RXSTALL TARGET: SCL sampled high after release; not a continuous waveform`n"
    $observed = Get-RxstallObservations $mode $fixture.controller $sampled
    Check ($observed.scl.sampled -and $observed.scl.evidence -eq 'discrete_input_samples') "$mode available SCL remains discrete samples"
    Check (-not $observed.scl.continuous_waveform_qualified) "$mode available SCL still not waveform qualification"
    Reject { Get-RxstallObservations $mode $fixture.controller ($sampled + "0.0 [WARN ] RXSTALL TARGET: SCL observer unavailable: GPIO3 clock disabled; no electrical verdict`n") } "$mode true flag contradicting unavailable warning rejected"
}
$fixture = Fixture 'long'
Reject { Get-RxstallObservations 'long' $fixture.controller ($fixture.target -replace 'elapsed_us=150273', 'elapsed_us=149999') } 'long mode requires measured 150 ms hold'
$fixture = Fixture 'short'
Reject { Get-RxstallObservations 'short' $fixture.controller ($fixture.target -replace 'elapsed_us=5128', 'elapsed_us=100000') } 'short hold must not reach controller deadline'
Write-Output "RXSTALL verdict controls: $passed passed"

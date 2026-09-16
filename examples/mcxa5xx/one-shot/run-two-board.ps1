# Build first; this runner flashes BOTH selected FRDM-MCXA577 boards.
param(
    [Parameter(Mandatory = $true)][string]$ControllerProbe,
    [Parameter(Mandatory = $true)][string]$TargetProbe,
    [ValidateRange(100,4000)][int]$SpeedKhz = 4000,
    [switch]$WriteRead
)
$ErrorActionPreference = 'Stop'
if ($ControllerProbe -eq $TargetProbe) { throw 'Select two different probes.' }
$examples = (Resolve-Path "$PSScriptRoot/..").Path
$stem = if ($WriteRead) { 'i2c-one-shot-write-read' } else { 'i2c-one-shot' }
$controllerElf = (Resolve-Path "$examples/target/thumbv8m.main-none-eabihf/release/$stem-controller").Path
$targetElf = (Resolve-Path "$examples/target/thumbv8m.main-none-eabihf/release/$stem-target").Path
$controllerVerdict = if ($WriteRead) { 'ONE-SHOT WRITE-READ CONTROLLER PASS: exact 4 bytes, RX quiescent, STOP observed' } else { 'ONE-SHOT CONTROLLER PASS: 16 bytes, DMA finished, STOP observed' }
$targetVerdict = if ($WriteRead) { 'ONE-SHOT WRITE-READ TARGET PASS: register 0x10, repeated START, no intermediate STOP, termination observed' } else { 'ONE-SHOT TARGET PASS: exact 16 bytes and STOP' }
$logs = Join-Path $examples ('target/i2c-one-shot-hil/' + [guid]::NewGuid().ToString('N'))
$null = New-Item -ItemType Directory -Path $logs
$processes = @{}
$repository = (Resolve-Path "$examples/../..").Path
# Selected source inventory, not a complete transitive build attestation. In
# particular, include the shared body and board decoder after their extraction.
$sources = @(
    Get-Item "$repository/embassy-mcxa/src/register_wait.rs", "$repository/embassy-mcxa/src/i2c/reset_sequence.rs"
    Get-ChildItem "$repository/embassy-mcxa/src/i2c/one_shot" -Recurse -File -Filter '*.rs'
    Get-ChildItem "$repository/embassy-mcxa/experimental/i2c-one-shot-*/src" -Recurse -File -Filter '*.rs'
    Get-ChildItem "$repository/examples/nxp-shared/src" -Recurse -File -Filter '*.rs'
    Get-ChildItem "$examples/src/bin/i2c_one_shot" -Recurse -File -Filter '*.rs'
    Get-ChildItem "$examples/src/bin/i2c-one-shot-*.rs" -File
    Get-Item "$examples/Cargo.toml", "$examples/Cargo.lock", "$repository/embassy-mcxa/Cargo.toml"
)
$result = [ordered]@{
    utc = [DateTime]::UtcNow.ToString('o'); status = 'fail'; write_read = [bool]$WriteRead
    controller_probe = $ControllerProbe; target_probe = $TargetProbe
    source_head = (& git -C $repository rev-parse HEAD)
    source_status = @(& git -C $repository status --porcelain=v1)
    source_hashes = @($sources | Sort-Object FullName -Unique | ForEach-Object {
        @{ path = $_.FullName; sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
    })
    controller_sha256 = (Get-FileHash -LiteralPath $controllerElf -Algorithm SHA256).Hash
    target_sha256 = (Get-FileHash -LiteralPath $targetElf -Algorithm SHA256).Hash
    controller_exit = $null; target_exit = $null
}

function Start-Board([string]$Role, [string]$Probe, [string]$Elf) {
    $arguments = @('run', '--chip', 'MCXA577', '--probe', $Probe,
        '--protocol', 'swd', '--speed', "$SpeedKhz", '--non-interactive',
        '--preverify', '--verify', '--scan-region', 'ram', ('"' + $Elf + '"'))
    $processes[$Role] = Start-Process -FilePath 'probe-rs' -ArgumentList $arguments -WindowStyle Hidden -PassThru -RedirectStandardOutput "$logs/$Role.out.log" -RedirectStandardError "$logs/$Role.err.log"
}
function Read-Log([string]$Role) {
    return [string](Get-Content -LiteralPath "$logs/$Role.out.log" -Raw -ErrorAction SilentlyContinue)
}

try {
    Start-Board 'controller' $ControllerProbe $controllerElf
    $deadline = [DateTime]::UtcNow.AddSeconds(30)
    while (-not ([string](Read-Log 'controller')).Contains('10s quiet window')) {
        if ($processes.controller.HasExited -or [DateTime]::UtcNow -gt $deadline) {
            throw 'Controller did not reach its startup window.'
        }
        Start-Sleep -Milliseconds 100
    }
    # Reset/flash the receiver after controller startup can no longer glitch it.
    Start-Board 'target' $TargetProbe $targetElf
    $deadline = [DateTime]::UtcNow.AddSeconds(25)
    while (-not ($processes.controller.HasExited -and $processes.target.HasExited)) {
        if ([DateTime]::UtcNow -gt $deadline) { throw 'Timed out waiting for both verdicts.' }
        Start-Sleep -Milliseconds 100
    }
    foreach ($role in @('controller', 'target')) {
        $processes[$role].WaitForExit()
        if ($processes[$role].ExitCode -ne 0) { throw "$role failed: exit $($processes[$role].ExitCode)" }
    }
    if (-not ([string](Read-Log 'controller')).Contains($controllerVerdict)) {
        throw 'Missing controller verdict.'
    }
    if (-not ([string](Read-Log 'target')).Contains($targetVerdict)) {
        throw 'Missing exact-data target verdict.'
    }
    $result.status = 'pass'
    Write-Output 'ONE-SHOT TWO-BOARD PASS'
} finally {
    foreach ($role in @('controller', 'target')) {
        if ($processes.ContainsKey($role) -and -not $processes[$role].HasExited) {
            Stop-Process -Id $processes[$role].Id
        }
        if ($processes.ContainsKey($role)) {
            $processes[$role].WaitForExit()
            $result["${role}_exit"] = $processes[$role].ExitCode
        }
        Get-Content -LiteralPath "$logs/$role.out.log" -ErrorAction SilentlyContinue
        Get-Content -LiteralPath "$logs/$role.err.log" -ErrorAction SilentlyContinue
    }
    $result | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath "$logs/result.json" -Encoding utf8
    Write-Output "Logs: $logs"
}

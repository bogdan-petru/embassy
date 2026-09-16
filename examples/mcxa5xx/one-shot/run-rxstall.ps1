#requires -Version 7.0
# This characterization flashes BOTH selected FRDM-MCXA577 boards.
# It never retries a failed mode and restores the ordinary example in finally.
param(
    [Parameter(Mandatory = $true)][string]$ControllerProbe,
    [Parameter(Mandatory = $true)][string]$TargetProbe,
    [ValidateRange(100, 4000)][int]$SpeedKhz = 4000
)
$ErrorActionPreference = 'Stop'
if ($ControllerProbe -eq $TargetProbe) { throw 'Select two different probes.' }
. "$PSScriptRoot/rxstall-verdicts.ps1"
$examples = (Resolve-Path "$PSScriptRoot/..").Path
$repository = (Resolve-Path "$examples/../..").Path
$logs = Join-Path $examples ('target/i2c-one-shot-rxstall/' + [guid]::NewGuid().ToString('N'))
$null = New-Item -ItemType Directory -Path $logs
$targetTriple = 'thumbv8m.main-none-eabihf'
$releaseDir = Join-Path $examples "target/$targetTriple/release"
$savedEnvironment = @{}
foreach ($name in @('I2C_ONE_SHOT_RXSTALL', 'I2C_ONE_SHOT_CHECK', 'RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'DEFMT_LOG', 'CARGO_TARGET_DIR')) {
    $savedEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
}
$run = [ordered]@{
    schema_version = 1
    started_utc = [DateTime]::UtcNow.ToString('o')
    controller_probe = $ControllerProbe
    target_probe = $TargetProbe
    speed_khz = $SpeedKhz
    scope = 'Checks sampled peripheral status and SCL input states while RX service is withheld; not continuous-waveform or electrical-timing qualification.'
    result = 'running'
    error = $null
    modes = [System.Collections.Generic.List[object]]::new()
    restore = [ordered]@{ result = 'not_needed'; error = $null }
}
$boardsTouched = $false
$primaryFailure = $null
$restoreFailure = $null

function Get-SourceSnapshot {
    $commit = & git -C $repository rev-parse HEAD
    if ($LASTEXITCODE -ne 0) { throw 'Cannot record source commit.' }
    $status = @(& git -C $repository status --porcelain=v1 --untracked-files=normal)
    if ($LASTEXITCODE -ne 0) { throw 'Cannot record source status.' }
    # Preserve the full repository status, but do not mistake unrelated C
    # examples/root build directories for dirty inputs to this experiment.
    # This is a declared review scope, not a complete transitive-source hash.
    $relevantPaths = @('embassy-mcxa', 'embassy-futures', 'embassy-sync', 'embassy-time', 'embassy-hal-internal',
        'examples/mcxa5xx/Cargo.toml', 'examples/mcxa5xx/build.rs', 'examples/mcxa5xx/.cargo',
        'examples/mcxa5xx/one-shot', 'examples/mcxa5xx/src/bin/i2c-one-shot-*.rs',
        'examples/mcxa5xx/src/bin/i2c_one_shot', 'examples/nxp-shared',
        'rust-toolchain.toml', 'ci.sh')
    $relevantStatus = @(& git -C $repository status --porcelain=v1 --untracked-files=all -- @relevantPaths)
    if ($LASTEXITCODE -ne 0) { throw 'Cannot record scoped source status.' }
    return [ordered]@{
        commit = [string]$commit; dirty = ($status.Count -ne 0); status = $status
        relevant_dirty = ($relevantStatus.Count -ne 0); relevant_status = $relevantStatus; relevant_paths = $relevantPaths
        # Selected shared sequence hashes, not a complete transitive-source hash.
        shared_sequence_hashes = @(
            Get-Item "$repository/embassy-mcxa/src/register_wait.rs", "$repository/embassy-mcxa/src/i2c/reset_sequence.rs" |
                ForEach-Object { @{ path = $_.FullName; sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash } }
        )
    }
}

function Stop-OwnedProcess([System.Diagnostics.Process]$Process) {
    if (-not $Process.HasExited) {
        try { $Process.Kill($true) } catch {
            # Exit between HasExited and Kill is harmless; failing to
            # stop a still-live child is not.
            if (-not $Process.HasExited) { throw }
        }
        if (-not $Process.WaitForExit(5000)) { throw "Cannot stop owned child PID $($Process.Id)." }
    }
    # Complete redirected output collection once the child has exited.
    $Process.WaitForExit()
}

function Invoke-BoundedProcess([string]$Program, [string[]]$Arguments, [string]$LogStem, [int]$TimeoutMs) {
    $child = Start-Process -FilePath $Program -ArgumentList $Arguments -WorkingDirectory $examples -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput "$LogStem.out.log" -RedirectStandardError "$LogStem.err.log"
    try {
        if (-not $child.WaitForExit($TimeoutMs)) { throw "$Program timed out after ${TimeoutMs}ms; see $LogStem.*.log" }
        $child.WaitForExit()
        if ($child.ExitCode -ne 0) { throw "$Program failed: exit $($child.ExitCode); see $LogStem.*.log" }
    } finally {
        Stop-OwnedProcess $child
        $child.Dispose()
        Get-Content -LiteralPath "$LogStem.out.log" -ErrorAction SilentlyContinue | Write-Host
        Get-Content -LiteralPath "$LogStem.err.log" -ErrorAction SilentlyContinue | Write-Host
    }
}

function Build-Bins([string[]]$Bins, [string]$LogStem) {
    $arguments = @('build', '--locked', '--release', '--target', $targetTriple, '--features', 'i2c-one-shot')
    foreach ($bin in $Bins) { $arguments += @('--bin', $bin) }
    Invoke-BoundedProcess 'cargo' $arguments $LogStem 180000
}

function Reset-Board([string]$Probe, [string]$LogStem) {
    Invoke-BoundedProcess 'probe-rs' @('reset', '--chip', 'MCXA577', '--probe', $Probe,
        '--protocol', 'swd', '--speed', "$SpeedKhz") $LogStem 20000
}

function Prepare-Board([string]$Probe, [string]$Elf, [string]$LogStem) {
    # Programming is setup, not part of the controller's quiet window. The
    # later `run --preverify --verify` still verifies this same ELF and resets
    # the board; preloading must never substitute for that fresh start.
    Invoke-BoundedProcess 'probe-rs' @('download', '--chip', 'MCXA577', '--probe', $Probe,
        '--protocol', 'swd', '--speed', "$SpeedKhz", '--non-interactive',
        '--preverify', '--verify', ('"' + $Elf + '"')) $LogStem 60000
}

function Start-Board([string]$Probe, [string]$Elf, [string]$LogStem) {
    $arguments = @('run', '--chip', 'MCXA577', '--probe', $Probe, '--protocol', 'swd', '--speed', "$SpeedKhz",
        '--non-interactive', '--preverify', '--verify', '--scan-region', 'ram', ('"' + $Elf + '"'))
    return Start-Process -FilePath 'probe-rs' -ArgumentList $arguments -WorkingDirectory $examples -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput "$LogStem.out.log" -RedirectStandardError "$LogStem.err.log"
}

function Read-BoardLog([string]$Directory, [string]$Role) {
    return [string](Get-Content -LiteralPath "$Directory/$Role.out.log" -Raw -ErrorAction SilentlyContinue)
}

function Run-Mode([string]$Mode) {
    $directory = Join-Path $logs $Mode
    $null = New-Item -ItemType Directory -Path $directory
    $modeRecord = [ordered]@{
        mode = $Mode; result = 'running'; error = $null; source = (Get-SourceSnapshot)
        directory = $directory; elf_sha256 = @{}; endpoints = @{}
        preparation = [ordered]@{ result = 'not_started'; elapsed_ms = $null }
        startup_budget_ms = 9000; target_ready_elapsed_ms = $null
    }
    $run.modes.Add($modeRecord)
    $processes = @{}
    try {
        $env:I2C_ONE_SHOT_RXSTALL = $Mode
        Build-Bins @('i2c-one-shot-rxstall-controller', 'i2c-one-shot-rxstall-target') "$directory/build"
        $controllerElf = (Resolve-Path "$releaseDir/i2c-one-shot-rxstall-controller").Path
        $targetElf = (Resolve-Path "$releaseDir/i2c-one-shot-rxstall-target").Path
        $modeRecord.elf_sha256.controller = (Get-FileHash -LiteralPath $controllerElf -Algorithm SHA256).Hash
        $modeRecord.elf_sha256.target = (Get-FileHash -LiteralPath $targetElf -Algorithm SHA256).Hash
        $modeRecord.source_after_build = Get-SourceSnapshot
        $script:boardsTouched = $true
        $preparation = [System.Diagnostics.Stopwatch]::StartNew()
        $modeRecord.preparation.result = 'running'
        try {
            Prepare-Board $ControllerProbe $controllerElf "$directory/controller-download"
            Prepare-Board $TargetProbe $targetElf "$directory/target-download"
            $modeRecord.preparation.result = 'pass'
        } catch {
            $modeRecord.preparation.result = 'fail'
            throw
        } finally {
            $modeRecord.preparation.elapsed_ms = $preparation.ElapsedMilliseconds
        }
        Reset-Board $ControllerProbe "$directory/controller-reset"
        Reset-Board $TargetProbe "$directory/target-reset"

        # The same conservative 9 s clock still starts BEFORE controller run,
        # including its preverification/reset/startup. Expensive programming of
        # both ELFs finished above; it cannot consume this quiet-window budget.
        # The target must still publish READY before the 10 s quiet window ends.
        # Per-board RTT timestamps cannot establish that cross-board ordering.
        $startup = [System.Diagnostics.Stopwatch]::StartNew()
        $processes.controller = Start-Board $ControllerProbe $controllerElf "$directory/controller"
        while (-not (Test-RxstallMarker (Read-BoardLog $directory 'controller') 'RXSTALL CONTROLLER: 10s quiet window')) {
            if ($processes.controller.HasExited -or $startup.ElapsedMilliseconds -ge 9000) {
                throw 'Controller did not enter its startup window in time to safely start the target.'
            }
            Start-Sleep -Milliseconds 50
        }
        $processes.target = Start-Board $TargetProbe $targetElf "$directory/target"
        $ready = $false
        while ($startup.ElapsedMilliseconds -lt 9000) {
            $ready = Test-RxstallMarker (Read-BoardLog $directory 'target') "RXSTALL TARGET READY: $Mode"
            if ($ready) { break }
            if ($processes.controller.HasExited -or $processes.target.HasExited) { break }
            Start-Sleep -Milliseconds 50
        }
        $modeRecord.target_ready_elapsed_ms = $startup.ElapsedMilliseconds
        Assert-RxstallStartup $ready $modeRecord.target_ready_elapsed_ms

        $completion = [System.Diagnostics.Stopwatch]::StartNew()
        while (-not ($processes.controller.HasExited -and $processes.target.HasExited)) {
            if ($completion.ElapsedMilliseconds -ge 25000) { throw 'Timed out waiting for both RXSTALL endpoints to exit.' }
            Start-Sleep -Milliseconds 50
        }
        $markers = Get-RxstallMarkers $Mode
        foreach ($role in @('controller', 'target')) {
            $processes[$role].WaitForExit()
            $log = Read-BoardLog $directory $role
            $modeRecord.endpoints[$role] = [ordered]@{
                exit_code = $processes[$role].ExitCode
                marker = $markers[$role]
                marker_seen = (Test-RxstallMarker $log $markers[$role])
            }
        }
        foreach ($role in @('controller', 'target')) {
            Assert-RxstallEndpoint $role $processes[$role].ExitCode (Read-BoardLog $directory $role) $markers[$role]
        }
        $modeRecord.observations = Get-RxstallObservations $Mode `
            (Read-BoardLog $directory 'controller') (Read-BoardLog $directory 'target')
        $modeRecord.result = 'pass'
        Write-Host "RXSTALL $($Mode.ToUpperInvariant()) TWO-BOARD PASS"
    } catch {
        $modeRecord.result = 'fail'
        $modeRecord.error = $_.Exception.Message
        throw
    } finally {
        foreach ($role in @('controller', 'target')) {
            if ($processes.ContainsKey($role)) {
                $wasRunning = -not $processes[$role].HasExited
                Stop-OwnedProcess $processes[$role]
                if (-not $modeRecord.endpoints.ContainsKey($role)) {
                    $modeRecord.endpoints[$role] = [ordered]@{
                        exit_code = $processes[$role].ExitCode; runner_terminated = $wasRunning
                    }
                }
                $processes[$role].Dispose()
            }
            Get-Content -LiteralPath "$directory/$role.out.log" -ErrorAction SilentlyContinue | Write-Host
            Get-Content -LiteralPath "$directory/$role.err.log" -ErrorAction SilentlyContinue | Write-Host
        }
    }
}

try {
    # Cargo gives encoded flags precedence over RUSTFLAGS. Clear them so the
    # warning/link flags below describe the build that is actually performed.
    Remove-Item Env:CARGO_ENCODED_RUSTFLAGS -ErrorAction SilentlyContinue
    $env:RUSTFLAGS = '-Dwarnings -C linker=flip-link -C link-arg=-Tlink.x -C link-arg=-Tdefmt.x -C link-arg=--nmagic'
    $env:DEFMT_LOG = 'info'
    $env:CARGO_TARGET_DIR = Join-Path $examples 'target'
    Remove-Item Env:I2C_ONE_SHOT_CHECK -ErrorAction SilentlyContinue
    $run.rustflags = $env:RUSTFLAGS
    $run.source = Get-SourceSnapshot
    Run-Mode 'short'
    Run-Mode 'long'
    $run.result = 'pass'
} catch {
    $primaryFailure = $_
    $run.result = 'fail'
    $run.error = $_.Exception.Message
} finally {
    if ($boardsTouched) {
        try {
            # Never restore a stale binary from another feature/env build.
            # This normal-pair success cannot erase a failed characterization.
            Remove-Item Env:I2C_ONE_SHOT_RXSTALL -ErrorAction SilentlyContinue
            Build-Bins @('i2c-one-shot-controller', 'i2c-one-shot-target') "$logs/restore-build"
            foreach ($bin in @('i2c-one-shot-controller', 'i2c-one-shot-target')) {
                if (-not (Test-Path -LiteralPath "$releaseDir/$bin" -PathType Leaf)) { throw "Missing restored ELF: $bin" }
            }
            $run.restore.elf_sha256 = @{
                controller = (Get-FileHash -LiteralPath "$releaseDir/i2c-one-shot-controller" -Algorithm SHA256).Hash
                target = (Get-FileHash -LiteralPath "$releaseDir/i2c-one-shot-target" -Algorithm SHA256).Hash
            }
            $run.restore.log = "$logs/restore.log"
            & "$PSScriptRoot/run-two-board.ps1" -ControllerProbe $ControllerProbe -TargetProbe $TargetProbe -SpeedKhz $SpeedKhz `
                *>&1 | Tee-Object -FilePath "$logs/restore.log" | Write-Host
            $run.restore.result = 'pass'
        } catch {
            $restoreFailure = $_
            $run.restore.result = 'fail'
            $run.restore.error = $_.Exception.Message
            $run.result = 'fail'
        }
    }
    foreach ($name in $savedEnvironment.Keys) {
        [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name], 'Process')
    }
    $run.completed_utc = [DateTime]::UtcNow.ToString('o')
    $run | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath "$logs/verdict.json" -Encoding utf8
    Write-Host "RXSTALL logs and verdict: $logs"
}
if ($primaryFailure) { throw $primaryFailure }
if ($restoreFailure) { throw $restoreFailure }
Write-Output 'RXSTALL SERVICE-WITHHOLDING CHECKS PASS; NORMAL PAIR RESTORED'

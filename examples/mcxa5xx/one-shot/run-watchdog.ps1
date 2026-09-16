#requires -Version 7.0
# This characterization flashes BOTH selected FRDM-MCXA577 boards.
# It never retries a failed mode and restores the ordinary example in finally.
param(
    [Parameter(Mandatory = $true)][string]$ControllerProbe,
    [Parameter(Mandatory = $true)][string]$TargetProbe,
    [ValidateRange(100, 4000)][int]$SpeedKhz = 4000,
    [ValidateSet('120', '140', '200')][string[]]$Modes = @('120', '140', '200'),
    [ValidateSet('none', 'wait', 'reset', 'abrupt', 'abrupt_idle')][string]$Recovery = 'none'
)
$ErrorActionPreference = 'Stop'
if ($ControllerProbe -eq $TargetProbe) { throw 'Select two different probes.' }
. "$PSScriptRoot/watchdog-verdicts.ps1"
Assert-WatchdogModes $Modes $Recovery
$examples = (Resolve-Path "$PSScriptRoot/..").Path
$repository = (Resolve-Path "$examples/../..").Path
$logs = Join-Path $examples ('target/i2c-one-shot-watchdog/' + [guid]::NewGuid().ToString('N'))
$null = New-Item -ItemType Directory -Path $logs
$targetTriple = 'thumbv8m.main-none-eabihf'
$releaseDir = Join-Path $examples "target/$targetTriple/release"
$savedEnvironment = @{}
foreach ($name in @('I2C_ONE_SHOT_WATCHDOG', 'I2C_ONE_SHOT_RECOVERY', 'I2C_ONE_SHOT_RXSTALL', 'I2C_ONE_SHOT_CHECK', 'RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'DEFMT_LOG', 'CARGO_TARGET_DIR')) {
    $savedEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
}
$run = [ordered]@{
    schema_version = 2
    started_utc = [DateTime]::UtcNow.ToString('o')
    controller_probe = $ControllerProbe
    target_probe = $TargetProbe
    speed_khz = $SpeedKhz
    requested_modes = @($Modes)
    recovery = $Recovery
    diagnostic_configuration_change = if ($Recovery -eq 'abrupt_idle') { 'MCFGR2.BUSIDLE = 2*(MCCR0.CLKLO + MCCR0.SETHOLD + 2), greater than MCCR0.CLKHI + 1; validated from readback.' } else { $null }
    full_sweep = ($Modes.Count -eq 3)
    scope = 'Checks SCL-only hardware pin-low thresholds and DMA quiescence while RX service is withheld, with a completing upper-threshold control. Default runs additionally require local controller MBF idle at return/after-wait and SCL high after peer release; BBF may remain set and no fresh default transfer is attempted. Diagnostic repair modes preserve their separate intervention/evidence rules. Board-selected elapsed-time windows are not precision bounds; no continuous-waveform or full bus-recovery qualification. Historical pre-fix failure logs are preserved, not accepted under the strengthened default rule.'
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
        mode = $Mode; recovery = $Recovery; result = 'running'; error = $null; source = (Get-SourceSnapshot)
        directory = $directory; elf_sha256 = @{}; endpoints = @{}
        startup_budget_ms = 19000; target_ready_elapsed_ms = $null; log_sha256 = @{}
    }
    $run.modes.Add($modeRecord)
    $processes = @{}
    try {
        $env:I2C_ONE_SHOT_WATCHDOG = $Mode
        $env:I2C_ONE_SHOT_RECOVERY = $Recovery
        Build-Bins @('i2c-one-shot-watchdog-controller', 'i2c-one-shot-watchdog-target') "$directory/build"
        $controllerElf = (Resolve-Path "$releaseDir/i2c-one-shot-watchdog-controller").Path
        $targetElf = (Resolve-Path "$releaseDir/i2c-one-shot-watchdog-target").Path
        $modeRecord.elf_sha256.controller = (Get-FileHash -LiteralPath $controllerElf -Algorithm SHA256).Hash
        $modeRecord.elf_sha256.target = (Get-FileHash -LiteralPath $targetElf -Algorithm SHA256).Hash
        $modeRecord.source_after_build = Get-SourceSnapshot
        $script:boardsTouched = $true
        Reset-Board $ControllerProbe "$directory/controller-reset"
        Reset-Board $TargetProbe "$directory/target-reset"

        # This clock starts BEFORE controller flash/startup, so a READY inside
        # 19 s is conservatively earlier than the end of its 20 s quiet window.
        # RTT timestamps are per-board and cannot establish this relationship.
        $startup = [System.Diagnostics.Stopwatch]::StartNew()
        $processes.controller = Start-Board $ControllerProbe $controllerElf "$directory/controller"
        while (-not (Test-RxstallMarker (Read-BoardLog $directory 'controller') 'WATCHDOG CONTROLLER: 20s quiet window')) {
            if ($processes.controller.HasExited -or $startup.ElapsedMilliseconds -ge 19000) {
                throw 'Controller did not enter its startup window in time to safely start the target.'
            }
            Start-Sleep -Milliseconds 50
        }
        $processes.target = Start-Board $TargetProbe $targetElf "$directory/target"
        $ready = $false
        while ($startup.ElapsedMilliseconds -lt 19000) {
            $ready = Test-RxstallMarker (Read-BoardLog $directory 'target') "WATCHDOG TARGET READY: $Mode"
            if ($ready) { break }
            if ($processes.controller.HasExited -or $processes.target.HasExited) { break }
            Start-Sleep -Milliseconds 50
        }
        $modeRecord.target_ready_elapsed_ms = $startup.ElapsedMilliseconds
        Assert-RxstallStartup $ready $modeRecord.target_ready_elapsed_ms 19000

        $completion = [System.Diagnostics.Stopwatch]::StartNew()
        while (-not ($processes.controller.HasExited -and $processes.target.HasExited)) {
            # A fault-mode controller may legitimately exit zero while the
            # target continues its measured hold. Only failure is fail-fast.
            foreach ($role in @('controller', 'target')) {
                if ($processes[$role].HasExited) {
                    $processes[$role].WaitForExit()
                    if ($processes[$role].ExitCode -ne 0) {
                        throw "$role failed during watchdog completion: exit $($processes[$role].ExitCode); see $directory/$role.*.log"
                    }
                }
            }
            if ($completion.ElapsedMilliseconds -ge 25000) { throw 'Timed out waiting for both WATCHDOG endpoints to exit.' }
            Start-Sleep -Milliseconds 50
        }
        $markers = Get-WatchdogMarkers $Mode
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
            Assert-WatchdogEndpoint $Mode $role $processes[$role].ExitCode (Read-BoardLog $directory $role)
        }
        $modeRecord.observations = Get-WatchdogObservations $Mode `
            (Read-BoardLog $directory 'controller') (Read-BoardLog $directory 'target') $Recovery
        $modeRecord.result = 'pass'
        Write-Host "WATCHDOG $($Mode.ToUpperInvariant()) TWO-BOARD PASS"
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
            foreach ($stream in @('out', 'err')) {
                $logFile = "$directory/$role.$stream.log"
                if (Test-Path -LiteralPath $logFile -PathType Leaf) {
                    $modeRecord.log_sha256["$role.$stream"] = (Get-FileHash -LiteralPath $logFile -Algorithm SHA256).Hash
                }
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
    Remove-Item Env:I2C_ONE_SHOT_RXSTALL -ErrorAction SilentlyContinue
    $run.rustflags = $env:RUSTFLAGS
    $run.source = Get-SourceSnapshot
    foreach ($mode in $Modes) {
        try {
            Run-Mode $mode
        } catch {
            # Preserve the first failure, then try the NEXT distinct mode.
            # Run-Mode resets both boards; this is never a retry of this case.
            if (-not $primaryFailure) {
                $primaryFailure = $_
                $run.error = $_.Exception.Message
            }
            $run.result = 'fail'
        }
    }
    $caseResults = @($run.modes | ForEach-Object { $_.result })
    if ($caseResults.Count -ne $Modes.Count) { throw 'Not every requested mode produced a case record.' }
    $run.result = Get-WatchdogAggregateResult $caseResults
    # A cleanup/orchestration exception must also remain a failure even if
    # the firmware had already set that individual case's result to pass.
    if ($primaryFailure) { $run.result = 'fail' }
} catch {
    if (-not $primaryFailure) {
        $primaryFailure = $_
        $run.error = $_.Exception.Message
    }
    $run.result = 'fail'
} finally {
    if ($boardsTouched) {
        try {
            # Never restore a stale binary from another feature/env build.
            # This normal-pair success cannot erase a failed characterization.
            Remove-Item Env:I2C_ONE_SHOT_WATCHDOG -ErrorAction SilentlyContinue
            Remove-Item Env:I2C_ONE_SHOT_RECOVERY -ErrorAction SilentlyContinue
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
    $run.log_sha256 = @{}
    Get-ChildItem -LiteralPath $logs -Filter '*.log' -File -Recurse | ForEach-Object {
        $relativeLog = [System.IO.Path]::GetRelativePath($logs, $_.FullName)
        $run.log_sha256[$relativeLog] = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
    }
    $run.completed_utc = [DateTime]::UtcNow.ToString('o')
    $run | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath "$logs/verdict.json" -Encoding utf8
    Write-Host "WATCHDOG logs and verdict: $logs"
}
if ($primaryFailure) { throw $primaryFailure }
if ($restoreFailure) { throw $restoreFailure }
if ($run.full_sweep) {
    Write-Output 'WATCHDOG FULL-SWEEP THRESHOLD CHECKS PASS; NORMAL PAIR RESTORED'
} else {
    Write-Output "WATCHDOG SELECTED MODES ($($Modes -join ', ')) PASS; NOT A FULL SWEEP; NORMAL PAIR RESTORED"
}

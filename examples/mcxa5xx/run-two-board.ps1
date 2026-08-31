# Two-board LPI2C suite runner (Windows / probe-rs).
#
# Runs `i2c-twoboard-controller` on one FRDM-MCXA577 while the other
# board runs `i2c-twoboard-target` or `i2c-twoboard-target-dma`
# (flash the target once with `probe-rs download` before this script).
# The rig is symmetric (P3_20<->P3_20 SDA, P3_21<->P3_21 SCL, GND<->GND
# with pull-ups), so which physical board plays which role is purely a
# probe-serial choice.
#
# De-raced ordering: the controller binary idles in a 2 s quiet window
# after boot (its own flash/reset can glitch a listening target into a
# stuck ADRSTALL stretch); this script resets the target inside that
# window so every run starts on a clean bus.
#
# Build recipe that matches the validated configuration:
#   $env:DEFMT_LOG = 'info'
#   cargo build --release --bin i2c-twoboard-controller `
#       --bin i2c-twoboard-target --bin i2c-twoboard-target-dma
# (`DEFMT_LOG=info` is also this crate's `.cargo/config.toml` default;
# higher log volume can fault the MCU-Link debug connection mid-run.)
# For probes that cannot hold a full three-phase session, set
# $env:SUITE_PHASES = 'async' (or 'dma,blocking'), rebuild the
# controller, and run each subset as its own session — the binary
# warns loudly when filtered and rejects unknown phase names.
#
# Exit 0 = every enabled phase passed (semihosting exit); nonzero =
# test failure (panic -> HardFault), timeout (124), probe loss, a
# controller that never reached its quiet window (2), or a target
# reset that failed (3) — the sync preconditions FAIL CLOSED rather
# than letting a mis-sequenced run masquerade as a verdict.
param(
    [int]$Seconds = 220,
    # Probe running the CONTROLLER binary (the RTT-heavy session).
    [string]$ControllerProbe = '1fc9:0143:ZS1PGS20MCJBC',
    # Probe hosting the TARGET board (only flashed/reset, never held).
    [string]$TargetProbe = '1fc9:0143:0YHQFRBFS4FIG',
    # SWD clock for the controller session, kHz. 4000 is deliberately
    # conservative for marginal probe/USB links.
    [int]$SpeedKhz = 4000,
    # Optional: path to the TARGET ELF. When set, a second probe-rs
    # session attaches to the target DURING the run and the captured
    # `[T]` RTT transcript is printed after it — wire-level evidence of
    # what the target saw (aborted serves, restarts, residue handling).
    # Build the target for it with (PowerShell):
    #   $env:DEFMT_LOG = 'info,embassy_mcxa::i2c::target=trace'
    #   $env:DEFMT_RTT_BUFFER_SIZE = '16384'
    #   cargo build --release --bin i2c-twoboard-target
    # for a transcript with per-serve abort lines.
    [string]$TargetElf = ''
)

$ctl = Join-Path $PSScriptRoot 'target\thumbv8m.main-none-eabihf\release\i2c-twoboard-controller'
$outFile = [System.IO.Path]::GetTempFileName()
$errFile = [System.IO.Path]::GetTempFileName()

$args = @('run','--chip','MCXA577','--preverify','--verify','--protocol','swd','--speed',"$SpeedKhz",
          '--probe',$ControllerProbe,'--non-interactive','--scan-region','ram',$ctl)
$proc = Start-Process -FilePath 'probe-rs' -ArgumentList $args `
    -RedirectStandardOutput $outFile -RedirectStandardError $errFile -PassThru -WindowStyle Hidden

# Reset the target once the controller actually reaches its quiet
# window — detected by its own log marker, not a fixed sleep, so a
# slow flash cannot let the reset fire early (a target reset BEFORE
# the window can glitch the bus the suite is about to sync on). FAIL
# CLOSED: no marker within 30 s means the controller session never
# came up sanely — kill it and report, rather than resetting blind
# and letting whatever follows pose as a test verdict.
$sawMarker = $false
$markerDeadline = (Get-Date).AddSeconds(30)
while ((Get-Date) -lt $markerDeadline) {
    $txt = Get-Content $outFile -Raw -ErrorAction SilentlyContinue
    if ($txt -and $txt.Contains('quiet window')) { $sawMarker = $true; break }
    # A controller session that already died (flash failure, probe
    # loss) will never print the marker — fail closed now, not in 30 s.
    if ($proc.HasExited) { break }
    Start-Sleep -Milliseconds 200
}
if (-not $sawMarker) {
    & taskkill /PID $proc.Id /T /F 2>&1 | Out-Null
    $proc.WaitForExit(5000) | Out-Null
    Get-Content $outFile -ErrorAction SilentlyContinue
    Get-Content $errFile -ErrorAction SilentlyContinue
    Write-Output "--- exit=2 (controller never reached its quiet window) ---"
    Remove-Item $outFile, $errFile -ErrorAction SilentlyContinue
    exit 2
}
# A beat into the 2 s window, clear of its opening edge.
Start-Sleep -Milliseconds 300
$null = probe-rs reset --chip MCXA577 --probe $TargetProbe --protocol swd --non-interactive 2>&1
$resetRc = $LASTEXITCODE
if ($resetRc -ne 0) {
    # An un-reset target means the suite syncs against stale state —
    # not a valid run. Fail closed.
    & taskkill /PID $proc.Id /T /F 2>&1 | Out-Null
    $proc.WaitForExit(5000) | Out-Null
    Get-Content $outFile -ErrorAction SilentlyContinue
    Get-Content $errFile -ErrorAction SilentlyContinue
    Write-Output "--- exit=3 (target reset failed, probe-rs exit $resetRc) ---"
    Remove-Item $outFile, $errFile -ErrorAction SilentlyContinue
    exit 3
}

# Optional live target transcript: attach AFTER the target reset (an
# attach never resets) and drain its RTT for the rest of the run.
$tgtProc = $null
$tgtOut = $null
$tgtErr = $null
if ($TargetElf -ne '') {
    $tgtOut = [System.IO.Path]::GetTempFileName()
    $tgtErr = [System.IO.Path]::GetTempFileName()
    $tgtArgs = @('attach','--chip','MCXA577','--protocol','swd','--speed',"$SpeedKhz",
                 '--probe',$TargetProbe,'--non-interactive','--scan-region','ram',$TargetElf)
    $tgtProc = Start-Process -FilePath 'probe-rs' -ArgumentList $tgtArgs `
        -RedirectStandardOutput $tgtOut -RedirectStandardError $tgtErr `
        -PassThru -WindowStyle Hidden
}

$timedOut = $false
if (-not $proc.WaitForExit($Seconds * 1000)) {
    $timedOut = $true
    & taskkill /PID $proc.Id /T /F 2>&1 | Out-Null
    $proc.WaitForExit(5000) | Out-Null
}

Get-Content $outFile -ErrorAction SilentlyContinue
Get-Content $errFile -ErrorAction SilentlyContinue
$code = try { $proc.ExitCode } catch { $null }
Write-Output "--- exit=$code timed_out=$timedOut ---"
Remove-Item $outFile, $errFile -ErrorAction SilentlyContinue

if ($null -ne $tgtProc) {
    # Give the target's tail a moment to flush, then stop the attach
    # and print the transcript. An attach that died early (marginal
    # probes drop RTT-heavy sessions) still yields its partial
    # transcript — but say so, with its stderr, rather than passing a
    # truncated capture off as complete.
    Start-Sleep -Milliseconds 500
    $attachDied = $tgtProc.HasExited
    & taskkill /PID $tgtProc.Id /T /F 2>&1 | Out-Null
    $tgtProc.WaitForExit(5000) | Out-Null
    Write-Output "--- target transcript ---"
    Get-Content $tgtOut -ErrorAction SilentlyContinue
    if ($attachDied) {
        Write-Output "--- WARNING: target attach exited before the run ended; transcript is PARTIAL ---"
        Get-Content $tgtErr -ErrorAction SilentlyContinue
    }
    Write-Output "--- end target transcript ---"
    Remove-Item $tgtOut, $tgtErr -ErrorAction SilentlyContinue
}

# Honor the documented contract: this script's exit code IS the run's
# verdict (0 = every enabled phase passed).
if ($timedOut) { exit 124 }
if ($code -is [int]) { exit $code }
exit 1

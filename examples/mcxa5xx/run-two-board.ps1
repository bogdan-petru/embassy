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
#   DEFMT_LOG=info cargo build --release --bin i2c-twoboard-controller \
#       --bin i2c-twoboard-target --bin i2c-twoboard-target-dma
# (`DEFMT_LOG=info` is also this crate's `.cargo/config.toml` default;
# higher log volume can fault the MCU-Link debug connection mid-run.)
# For probes that cannot hold a full three-phase session, build the
# controller with SUITE_PHASES=async / SUITE_PHASES=dma,blocking and
# run each subset as its own session — the binary warns loudly when
# filtered.
#
# Exit 0 = every enabled phase passed (semihosting exit); nonzero =
# test failure (panic -> HardFault), timeout, or probe loss.
param(
    [int]$Seconds = 220,
    # Probe running the CONTROLLER binary (the RTT-heavy session).
    [string]$ControllerProbe = '1fc9:0143:ZS1PGS20MCJBC',
    # Probe hosting the TARGET board (only flashed/reset, never held).
    [string]$TargetProbe = '1fc9:0143:0YHQFRBFS4FIG',
    # SWD clock for the controller session, kHz. 4000 is deliberately
    # conservative for marginal probe/USB links.
    [int]$SpeedKhz = 4000
)

$ctl = Join-Path $PSScriptRoot 'target\thumbv8m.main-none-eabihf\release\i2c-twoboard-controller'
$outFile = [System.IO.Path]::GetTempFileName()
$errFile = [System.IO.Path]::GetTempFileName()

$args = @('run','--chip','MCXA577','--preverify','--verify','--protocol','swd','--speed',"$SpeedKhz",
          '--probe',$ControllerProbe,'--non-interactive',$ctl)
$proc = Start-Process -FilePath 'probe-rs' -ArgumentList $args `
    -RedirectStandardOutput $outFile -RedirectStandardError $errFile -PassThru -NoNewWindow

# Reset the target once the controller actually reaches its quiet
# window — detected by its own log marker, not a fixed sleep, so a
# slow flash cannot let the reset fire early (a target reset BEFORE
# the window can glitch the bus the suite is about to sync on). Falls
# back after 30 s (probe never came up: the reset is then harmless).
$markerDeadline = (Get-Date).AddSeconds(30)
while ((Get-Date) -lt $markerDeadline) {
    $txt = Get-Content $outFile -Raw -ErrorAction SilentlyContinue
    if ($txt -and $txt.Contains('quiet window')) { break }
    Start-Sleep -Milliseconds 200
}
# A beat into the 2 s window, clear of its opening edge.
Start-Sleep -Milliseconds 300
probe-rs reset --chip MCXA577 --probe $TargetProbe --protocol swd --non-interactive 2>&1 | Out-Null

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

# Honor the documented contract: this script's exit code IS the run's
# verdict (0 = every enabled phase passed).
if ($timedOut) { exit 124 }
if ($code -is [int]) { exit $code }
exit 1

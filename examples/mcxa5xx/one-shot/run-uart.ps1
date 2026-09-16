# Build first. Flashes the two selected MCXA577 boards, one direction per run.
param(
    [Parameter(Mandatory)][string]$SenderProbe,
    [Parameter(Mandatory)][string]$ReceiverProbe,
    [ValidateRange(100,4000)][int]$SpeedKhz = 4000
)
$ErrorActionPreference = 'Stop'
if ($SenderProbe -eq $ReceiverProbe) { throw 'Select two different probes.' }
$examples = (Resolve-Path "$PSScriptRoot/..").Path
$repo = (Resolve-Path "$examples/../..").Path
$binaryRoot = "$examples/target/uart-contract-pilot/thumbv8m.main-none-eabihf/release"
$elf = @{
    sender = (Resolve-Path "$binaryRoot/uart-one-shot-sender").Path
    receiver = (Resolve-Path "$binaryRoot/uart-one-shot-receiver").Path
}
$probes = @{ sender = $SenderProbe; receiver = $ReceiverProbe }
$logs = Join-Path $examples ('target/uart-one-shot-hil/' + [guid]::NewGuid().ToString('N'))
$null = New-Item -ItemType Directory -Path $logs
. "$PSScriptRoot/rxstall-verdicts.ps1" # Reuse tested exact INFO-marker parsing.
$record = [ordered]@{
    utc = [DateTime]::UtcNow.ToString('o'); status = 'RUNNING'
    sender_probe = $SenderProbe; receiver_probe = $ReceiverProbe
    wiring = 'LPUART5: sender P0_25 TX -> receiver P0_24 RX; common ground'
    speed_khz = $SpeedKhz; source_head = (& git -C $repo rev-parse HEAD)
    probe_version = (& probe-rs --version); elf = @{}
}
foreach ($role in @('sender','receiver')) {
    $record.elf[$role] = @{ path = $elf[$role]; sha256 = (Get-FileHash -LiteralPath $elf[$role] -Algorithm SHA256).Hash }
}
# Include untracked pilot sources, not only the tracked diff/HEAD.
$sources = @(
    "$repo/embassy-mcxa/src/register_wait.rs",
    "$repo/embassy-mcxa/src/lpuart/one_shot.rs",
    "$repo/embassy-mcxa/src/lpuart/one_shot",
    "$repo/embassy-mcxa/experimental/uart-one-shot-core/src",
    "$repo/embassy-mcxa/experimental/uart-one-shot-interface/src",
    "$repo/embassy-mcxa/experimental/uart-one-shot-vocabulary/src",
    "$repo/examples/nxp-shared/src",
    "$examples/src/bin/uart-one-shot-sender.rs", "$examples/src/bin/uart-one-shot-receiver.rs"
)
$record.source_hashes = @(Get-ChildItem -LiteralPath $sources -Recurse -File |
    Where-Object Extension -eq '.rs' | ForEach-Object {
        @{ path = $_.FullName; sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
    })
& git -C $repo status --short | Set-Content -LiteralPath "$logs/source-status.log"
$processes = @{}
function Read-Log([string]$Role) {
    return [string](Get-Content -LiteralPath "$logs/$Role.out.log" -Raw -ErrorAction SilentlyContinue)
}
function Stop-Owned([System.Diagnostics.Process]$Child) {
    if (-not $Child.HasExited) {
        try { $Child.Kill($true) } catch { if (-not $Child.HasExited) { throw } }
    }
    if (-not $Child.WaitForExit(5000)) { throw "Cannot stop owned child $($Child.Id)." }
    $Child.WaitForExit()
}
function Board-Arguments([string]$Verb, [string]$Role) {
    $argsForBoard = @($Verb, '--chip', 'MCXA577', '--probe', $probes[$Role],
        '--protocol', 'swd', '--speed', "$SpeedKhz", '--non-interactive', '--preverify', '--verify')
    if ($Verb -eq 'run') { $argsForBoard += @('--scan-region', 'ram') }
    return $argsForBoard + @('"' + $elf[$Role] + '"')
}
function Start-Board([string]$Role) {
    $processes[$Role] = Start-Process -FilePath 'probe-rs' -ArgumentList (Board-Arguments 'run' $Role) `
        -WorkingDirectory $examples -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput "$logs/$Role.out.log" -RedirectStandardError "$logs/$Role.err.log"
}
try {
    # Preload before either startup deadline begins; fresh run still verifies
    # the same ELF and resets each board. No marker from preparation is used.
    foreach ($role in @('sender','receiver')) {
        $child = Start-Process -FilePath 'probe-rs' -ArgumentList (Board-Arguments 'download' $role) `
            -WorkingDirectory $examples -WindowStyle Hidden -PassThru `
            -RedirectStandardOutput "$logs/$role-prepare.out.log" -RedirectStandardError "$logs/$role-prepare.err.log"
        try {
            if (-not $child.WaitForExit(60000)) { throw "$role programming timed out." }
            $child.WaitForExit()
            if ($child.ExitCode -ne 0) { throw "$role programming failed: exit $($child.ExitCode)." }
        } finally { Stop-Owned $child; $child.Dispose() }
    }
    $readyClock = [Diagnostics.Stopwatch]::StartNew()
    Start-Board 'sender'
    while (-not (Test-RxstallMarker (Read-Log 'sender') 'UART PILOT SENDER: 10s quiet window; LPUART5/P0_25, 115200 8N1')) {
        if ($processes.sender.HasExited -or $readyClock.ElapsedMilliseconds -ge 9000) { throw 'Sender startup marker missing inside readiness budget.' }
        Start-Sleep -Milliseconds 50
    }
    Start-Board 'receiver'
    while (-not (Test-RxstallMarker (Read-Log 'receiver') 'UART PILOT RECEIVER READY: LPUART5/P0_24, 115200 8N1')) {
        if ($processes.sender.HasExited -or $processes.receiver.HasExited -or $readyClock.ElapsedMilliseconds -ge 9000) { throw 'Receiver readiness missed the sender quiet window.' }
        Start-Sleep -Milliseconds 50
    }
    Assert-RxstallStartup $true $readyClock.ElapsedMilliseconds 9000
    $record.receiver_ready_ms_after_sender_launch = $readyClock.ElapsedMilliseconds
    $deadline = [DateTime]::UtcNow.AddSeconds(30)
    while (-not ($processes.sender.HasExited -and $processes.receiver.HasExited)) {
        if ([DateTime]::UtcNow -ge $deadline) { throw 'UART verdict deadline exceeded.' }
        Start-Sleep -Milliseconds 50
    }
    foreach ($role in @('sender','receiver')) { $processes[$role].WaitForExit() }
    Assert-RxstallEndpoint 'sender' $processes.sender.ExitCode (Read-Log 'sender') `
        'UART PILOT SENDER PASS: 16 bytes, DMA quiescent, transmission complete'
    Assert-RxstallEndpoint 'receiver' $processes.receiver.ExitCode (Read-Log 'receiver') `
        'UART PILOT RECEIVER PASS: exact 16 bytes, no extra bytes for 20ms'
    $record.status = 'PASS'
} catch {
    $record.status = 'FAIL'; $record.failure = $_.Exception.Message
    throw
} finally {
    $cleanupErrors = @()
    foreach ($role in @('sender','receiver')) {
        if ($processes.ContainsKey($role)) {
            try {
                Stop-Owned $processes[$role]
                $record["${role}_exit"] = $processes[$role].ExitCode
            } catch { $cleanupErrors += "$role cleanup: $($_.Exception.Message)" }
            finally { $processes[$role].Dispose() }
        }
        Get-Content -LiteralPath "$logs/$role.out.log" -ErrorAction SilentlyContinue | Write-Host
        Get-Content -LiteralPath "$logs/$role.err.log" -ErrorAction SilentlyContinue | Write-Host
    }
    if ($cleanupErrors.Count) { $record.status = 'FAIL'; $record.cleanup_errors = $cleanupErrors }
    $record | ConvertTo-Json -Depth 7 | Set-Content -LiteralPath "$logs/result.json"
    Write-Output "Logs: $logs"
    if ($cleanupErrors.Count) { throw ($cleanupErrors -join '; ') }
}
Write-Output 'UART TWO-BOARD PASS'

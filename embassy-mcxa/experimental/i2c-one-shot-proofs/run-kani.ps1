#Requires -Version 7.0
<#
.SYNOPSIS
Run both complete Kani suites in the existing WSL installation, saving evidence.
.NOTES
No installation, source mutation, harness filtering, or weakened safety flags.
The Linux timeout owns only this invocation; no distribution-wide shutdown is used.
Use -SelfTest to test the result parser without WSL or Kani.
#>
[CmdletBinding()]
param(
    [string] $Distribution = 'Ubuntu',
    [string] $LinuxUser = 'kani',
    [string] $TargetDirectory = '/home/kani/target/i2c-one-shot-l3',
    [string] $LogRoot = (Join-Path ([IO.Path]::GetTempPath()) ('i2c-kani-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0, 8))),
    [ValidateRange(30, 7200)] [int] $TimeoutSeconds = 1800,
    [switch] $SelfTest
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Assert-KaniResult {
    param([string] $Text, [int] $ExitCode, [bool] $TimedOut, [string[]] $Expected)
    if ($TimedOut -or $ExitCode -ne 0) { throw "Kani process failed: exit=$ExitCode, timeout=$TimedOut" }
    if ($Text -match '(?im)^\s*(?:VERIFICATION:-\s*(?!SUCCESSFUL\s*$)\S+|Error:|error(?:\[|:)|.*Status:\s*(?:FAILURE|UNKNOWN|UNDETERMINED))') {
        throw 'Kani output contains an error or non-successful verification status'
    }
    $summaries = [regex]::Matches($Text, '(?m)^Complete - (\d+) successfully verified harnesses, (\d+) failures, (\d+) total\.\s*$')
    if ($summaries.Count -ne 1) { throw 'Expected exactly one complete harness summary' }
    $summary = $summaries[0]
    if ([int]$summary.Groups[1].Value -ne $Expected.Count -or [int]$summary.Groups[2].Value -ne 0 -or [int]$summary.Groups[3].Value -ne $Expected.Count) {
        throw 'Wrong success/failure/total harness counts'
    }
    $started = [regex]::Matches($Text, '(?m)^Checking harness ([A-Za-z_][A-Za-z_0-9:]*)\.\.\.\s*$')
    $names = @($started | ForEach-Object { ($_.Groups[1].Value -split '::')[-1] })
    if ($names.Count -ne $Expected.Count -or @($names | Sort-Object -Unique).Count -ne $Expected.Count) {
        throw 'Missing or duplicate harness executions'
    }
    if (@(Compare-Object ($names | Sort-Object) ($Expected | Sort-Object)).Count -ne 0) {
        throw 'Executed harness names do not match the required complete suite'
    }
    if ([regex]::Matches($Text, '(?m)^VERIFICATION:- SUCCESSFUL\s*$').Count -ne $Expected.Count) {
        throw 'Unexpected number of verification result blocks'
    }
    # Tie every SUCCESSFUL result and zero-failure check summary to its own harness.
    for ($i = 0; $i -lt $started.Count; $i++) {
        $end = if ($i + 1 -lt $started.Count) { $started[$i + 1].Index } else { $summary.Index }
        $segment = $Text.Substring($started[$i].Index, $end - $started[$i].Index)
        $checks = [regex]::Matches($segment, '(?m)^\s*\*\* 0 of ([1-9][0-9]*) failed(?: \(([0-9]+) unreachable\))?\s*$')
        if ([regex]::Matches($segment, '(?m)^VERIFICATION:- SUCCESSFUL\s*$').Count -ne 1 -or
            $checks.Count -ne 1) {
            throw "Harness $($names[$i]) lacks exactly one successful nonempty check result"
        }
        if ($checks[0].Groups[2].Success -and [long]$checks[0].Groups[2].Value -ge [long]$checks[0].Groups[1].Value) {
            throw "Harness $($names[$i]) has no reachable checks"
        }
    }
    return [ordered]@{ successful = $Expected.Count; failures = 0; total = $Expected.Count; harnesses = $names }
}

function Test-ResultParser {
    $expected = @('first', 'second')
    $good = "Checking harness proofs::first...`n ** 0 of 10 failed`nVERIFICATION:- SUCCESSFUL`nChecking harness second...`n ** 0 of 2 failed`nVERIFICATION:- SUCCESSFUL`nComplete - 2 successfully verified harnesses, 0 failures, 2 total.`n"
    $null = Assert-KaniResult $good 0 $false $expected
    $null = Assert-KaniResult ($good.Replace('0 of 10 failed', '0 of 10 failed (2 unreachable)')) 0 $false $expected
    $cases = @(
        @{ text = $good; code = 1; timeout = $false },
        @{ text = $good; code = 0; timeout = $true },
        @{ text = ''; code = 0; timeout = $false },
        @{ text = $good.Replace('2 successfully', '1 successfully'); code = 0; timeout = $false },
        @{ text = $good.Replace('0 failures', '1 failures'); code = 0; timeout = $false },
        @{ text = $good.Replace('2 total', '3 total'); code = 0; timeout = $false },
        @{ text = $good.Replace('harness second', 'harness other'); code = 0; timeout = $false },
        @{ text = $good.Replace('harness second', 'harness proofs::first'); code = 0; timeout = $false },
        @{ text = $good.Replace(' ** 0 of 2 failed', ' ** 1 of 2 failed'); code = 0; timeout = $false },
        @{ text = $good.Replace(' ** 0 of 2 failed', ' ** 0 of 0 failed'); code = 0; timeout = $false },
        @{ text = $good.Replace('0 of 2 failed', '0 of 2 failed (2 unreachable)'); code = 0; timeout = $false },
        @{ text = $good.Replace('0 of 2 failed', '0 of 2 failed (3 unreachable)'); code = 0; timeout = $false },
        @{ text = $good.Replace('0 of 2 failed', '0 of 2 failed (one unreachable)'); code = 0; timeout = $false },
        @{ text = $good.Replace('0 of 2 failed', '0 of 2 failed (1 unreachable'); code = 0; timeout = $false },
        @{ text = $good.Replace('VERIFICATION:- SUCCESSFUL', 'VERIFICATION:- FAILED'); code = 0; timeout = $false },
        @{ text = $good.Replace('VERIFICATION:- SUCCESSFUL', ''); code = 0; timeout = $false },
        @{ text = $good + "error: linker failed`n"; code = 0; timeout = $false },
        @{ text = $good + " - Status: UNKNOWN`n"; code = 0; timeout = $false },
        @{ text = "VERIFICATION:- SUCCESSFUL`n" + $good; code = 0; timeout = $false },
        @{ text = $good + $good; code = 0; timeout = $false }
    )
    foreach ($case in $cases) {
        $rejected = $false
        try { $null = Assert-KaniResult $case.text $case.code $case.timeout $expected } catch { $rejected = $true }
        if (-not $rejected) { throw "Parser accepted bad fixture: $($case | ConvertTo-Json -Compress)" }
    }
    Write-Output "Kani result parser: $($cases.Count + 2) fixtures passed (two positive, $($cases.Count) negative)."
}

if ($SelfTest) { Test-ResultParser; return }
if (-not $IsWindows) { throw 'This wrapper is for PowerShell on Windows with an existing WSL installation.' }
if ($LinuxUser -notmatch '^[a-z_][a-z0-9_-]*$' -or $TargetDirectory -notmatch '^/[^\r\n]+$') { throw 'Invalid Linux user or absolute target directory.' }
$LogRoot = [IO.Path]::GetFullPath($LogRoot)
if (Test-Path -LiteralPath $LogRoot) { throw "Evidence directory already exists; choose a fresh -LogRoot: $LogRoot" }
$null = New-Item -ItemType Directory -Path $LogRoot

function Save-Json([string] $Name, $Value) {
    ConvertTo-Json -InputObject $Value -Depth 12 | Set-Content -LiteralPath (Join-Path $LogRoot $Name) -Encoding utf8
}

$experimental = Split-Path -Parent $PSScriptRoot
function Get-SourceInventory {
    $files = foreach ($package in @('i2c-one-shot-proofs', 'i2c-one-shot-core', 'i2c-one-shot-vocabulary', 'i2c-one-shot-interface')) {
        $directory = Join-Path $experimental $package
        foreach ($name in @('Cargo.toml', 'Cargo.lock')) {
            $path = Join-Path $directory $name
            if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Required input missing: $path" }
            Get-Item -LiteralPath $path
        }
        foreach ($name in @('build.rs', 'run-kani.ps1')) {
            $path = Join-Path $directory $name
            if (Test-Path -LiteralPath $path -PathType Leaf) { Get-Item -LiteralPath $path }
        }
        foreach ($name in @('src', 'tests')) {
            $path = Join-Path $directory $name
            if (Test-Path -LiteralPath $path -PathType Container) { Get-ChildItem -LiteralPath $path -Recurse -File }
        }
    }
    foreach ($file in $files | Sort-Object FullName -Unique) {
        [ordered]@{ path = [IO.Path]::GetRelativePath($experimental, $file.FullName).Replace('\', '/'); sha256 = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash; bytes = $file.Length }
    }
}

$commands = [Collections.Generic.List[object]]::new()
function Invoke-Captured {
    param([string] $Name, [string[]] $Arguments, [int] $Seconds)
    $start = [DateTimeOffset]::UtcNow
    $entry = [ordered]@{ name = $Name; executable = 'wsl.exe'; arguments = $Arguments; started_utc = $start.ToString('o'); timeout_seconds = $Seconds; pid = $null; exit_code = $null; timed_out = $false }
    $commands.Add($entry)
    Save-Json 'commands.json' $commands
    $info = [Diagnostics.ProcessStartInfo]::new('wsl.exe')
    $info.UseShellExecute = $false
    $info.CreateNoWindow = $true
    $info.RedirectStandardOutput = $true
    $info.RedirectStandardError = $true
    foreach ($argument in $Arguments) { $info.ArgumentList.Add($argument) }
    $process = [Diagnostics.Process]::new()
    $process.StartInfo = $info
    try {
        if (-not $process.Start()) { throw 'Could not start wsl.exe' }
        $entry.pid = $process.Id
        $stdout = $process.StandardOutput.ReadToEndAsync()
        $stderr = $process.StandardError.ReadToEndAsync()
        if (-not $process.WaitForExit(($Seconds + 60) * 1000)) {
            $entry.timed_out = $true
            # This handle belongs to our launch, never another user's WSL session.
            $process.Kill($true)
            if (-not $process.WaitForExit(15000)) { throw 'Owned wsl.exe child did not exit after termination' }
        }
        $entry.exit_code = $process.ExitCode
        if ($process.ExitCode -in @(124, 137)) { $entry.timed_out = $true }
        if (-not [Threading.Tasks.Task]::WaitAll(@($stdout, $stderr), 15000)) { throw 'Timed out collecting child output' }
        $outText = $stdout.GetAwaiter().GetResult()
        $errText = $stderr.GetAwaiter().GetResult()
        [IO.File]::WriteAllText((Join-Path $LogRoot "$Name.stdout.log"), $outText)
        [IO.File]::WriteAllText((Join-Path $LogRoot "$Name.stderr.log"), $errText)
        return [ordered]@{ text = $outText + "`n" + $errText; exit_code = $entry.exit_code; timed_out = $entry.timed_out }
    } finally {
        $entry['elapsed_seconds'] = ([DateTimeOffset]::UtcNow - $start).TotalSeconds
        Save-Json 'commands.json' $commands
        $process.Dispose()
    }
}

$linuxHome = "/home/$LinuxUser"
$prefix = @('--distribution', $Distribution, '--user', $LinuxUser, '--cd', $linuxHome, '--exec', '/usr/bin/env',
    '-u', 'CARGO_ENCODED_RUSTFLAGS', '-u', 'RUSTFLAGS', '-u', 'RUSTDOCFLAGS', '-u', 'CARGO_BUILD_TARGET', '-u', 'CARGO_TARGET_DIR',
    "PATH=$linuxHome/.cargo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin", 'RUSTUP_TOOLCHAIN=stable')
$manifest = Join-Path $PSScriptRoot 'Cargo.toml'
if ($manifest -notmatch '^([A-Za-z]):[\\/](.*)$') { throw 'The checked-out manifest must be on a local Windows drive.' }
$linuxManifest = '/mnt/' + $Matches[1].ToLowerInvariant() + '/' + $Matches[2].Replace('\', '/')
$expected = [ordered]@{
    library = @('fault_priority_for_every_snapshot', 'completion_observation_never_swallows_a_fault', 'all_input_sizes_are_rejected_or_fit_one_receive_command')
    vocabulary = @('start_admission_preserves_address', 'stop_uses_current_fifo_state', 'read_readiness_is_pure', 'observations_preserve_status_and_wake', 'tx_arm_preserves_input_and_error', 'rx_arm_preserves_input_and_error', 'tx_poll_preserves_owner_and_outcome', 'rx_poll_preserves_owner_and_outcome', 'forgetting_pending_owners_does_not_complete')
}
$result = [ordered]@{ accepted = $false; started_utc = [DateTimeOffset]::UtcNow.ToString('o'); tools = [ordered]@{}; suites = [ordered]@{}; source_unchanged = $false; lockfiles_unchanged = $false; failure = $null; platform = 'x86_64-unknown-linux-gnu under WSL; not an ARM binary proof' }
$before = @()
$mutex = [Threading.Mutex]::new($false, 'Local\I2cOneShotKaniRunner')
$owned = $false
try {
    try { $owned = $mutex.WaitOne(0) } catch [Threading.AbandonedMutexException] {
        $owned = $true
        throw 'A previous runner terminated unexpectedly. Check its owned processes and evidence before retrying.'
    }
    if (-not $owned) { throw 'Another copy of this Kani runner is active.' }
    $before = @(Get-SourceInventory)
    Save-Json 'source-before.json' $before
    foreach ($version in @(
        @{ name = 'cargo-version'; args = @("$linuxHome/.cargo/bin/cargo", '--version') },
        @{ name = 'rustc-dispatcher-version'; args = @("$linuxHome/.cargo/bin/rustc", '-vV') },
        @{ name = 'kani-version'; args = @("$linuxHome/.cargo/bin/cargo", 'kani', '--version') },
        @{ name = 'linux-version'; args = @('/usr/bin/uname', '-a') }
    )) {
        $capture = Invoke-Captured $version.name ($prefix + @('/usr/bin/timeout', '--signal=TERM', '--kill-after=15s', '30s') + $version.args) 30
        if ($capture.exit_code -ne 0 -or $capture.timed_out) { throw "Version probe failed: $($version.name)" }
        $result.tools[$version.name] = $capture.text.Trim()
    }
    foreach ($suite in $expected.Keys) {
        $selection = if ($suite -eq 'library') { @('--lib') } else { @('--bin', 'vocabulary') }
        $arguments = $prefix + @('/usr/bin/timeout', '--signal=TERM', '--kill-after=15s', "${TimeoutSeconds}s", "$linuxHome/.cargo/bin/cargo", 'kani', '--manifest-path', $linuxManifest, '--target-dir', $TargetDirectory, '--output-format', 'regular') + $selection
        $capture = Invoke-Captured $suite $arguments $TimeoutSeconds
        $result.suites[$suite] = Assert-KaniResult $capture.text $capture.exit_code $capture.timed_out $expected[$suite]
        $result.tools["$suite-backends"] = @([regex]::Matches($capture.text, '(?m)^(?:CBMC version|Solving with) .+$') | ForEach-Object { $_.Value.Trim() } | Sort-Object -Unique)
        Write-Output "$suite`: $($expected[$suite].Count) harnesses verified."
    }
} catch {
    $result.failure = $_.Exception.Message
} finally {
    try {
        $after = @(Get-SourceInventory)
        Save-Json 'source-after.json' $after
        $beforeRows = @($before | ForEach-Object { "$($_.path)|$($_.sha256)|$($_.bytes)" })
        $afterRows = @($after | ForEach-Object { "$($_.path)|$($_.sha256)|$($_.bytes)" })
        $differences = @(if ($before.Count -gt 0) { Compare-Object $beforeRows $afterRows } else { 'missing initial inventory' })
        Save-Json 'source-differences.json' $differences
        $result.source_unchanged = $differences.Count -eq 0
        $lockDifferences = @($differences | Where-Object { "$_" -match 'Cargo\.lock' })
        $result.lockfiles_unchanged = $before.Count -gt 0 -and $lockDifferences.Count -eq 0
        if (-not $result.source_unchanged) { $result.failure = 'Source or lockfile changed during verification; results are not accepted.' }
    } catch { $result.failure = "Inventory verification failed: $($_.Exception.Message)" }
    $result.accepted = $null -eq $result.failure -and $result.suites.Count -eq 2 -and $result.source_unchanged -and $result.lockfiles_unchanged
    $result['finished_utc'] = [DateTimeOffset]::UtcNow.ToString('o')
    Save-Json 'result.json' $result
    if ($owned) { $mutex.ReleaseMutex() }
    $mutex.Dispose()
}
if (-not $result.accepted) { throw "Kani suite rejected: $($result.failure). Evidence: $LogRoot" }
Write-Output "Kani suites accepted: 3 library + 9 vocabulary harnesses. Evidence: $LogRoot"

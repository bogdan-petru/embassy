#Requires -Version 7.0
<#
.SYNOPSIS
Show that four selected L3 proofs reject deliberately broken real vocabulary code.
.NOTES
These filtered negative controls do not replace the complete 3+9-harness suite.
Every control gets a fresh private four-crate copy, a passing baseline, and one
exact source mutation. The checkout is only read. Evidence and copies are kept.
#>
[CmdletBinding()]
param(
    [string] $Distribution = 'Ubuntu',
    [string] $LinuxUser = 'kani',
    [string] $TargetDirectory = '/home/kani/target/i2c-one-shot-l3-controls',
    [string] $LogRoot = (Join-Path ([IO.Path]::GetTempPath()) ('i2c-l3-controls-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0, 8))),
    [ValidateRange(30, 7200)] [int] $TimeoutSeconds = 600,
    [switch] $SelfTest
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Assert-ControlLog {
    param([string] $Text, [int] $ExitCode, [bool] $TimedOut, [string] $Harness, [string[]] $Failures = @())
    $negative = $Failures.Count -gt 0
    $expectedExit = if ($negative) { 1 } else { 0 }
    if ($TimedOut -or $ExitCode -ne $expectedExit) { throw 'Unexpected process result or timeout' }
    if ($Text -match '(?im)^\s*(?:error(?:\[|:)|.*Status:\s*(?:UNKNOWN|UNDETERMINED))') { throw 'Compilation or indeterminate verification result' }
    $starts = [regex]::Matches($Text, '(?m)^Checking harness ([A-Za-z_][A-Za-z_0-9:]*)\.\.\.\s*$')
    if ($starts.Count -ne 1 -or $starts[0].Groups[1].Value -cne $Harness) { throw 'Wrong or duplicated selected harness' }
    $summaries = [regex]::Matches($Text, '(?m)^Complete - (\d+) successfully verified harnesses, (\d+) failures, (\d+) total\.\s*$')
    if ($summaries.Count -ne 1 -or [int]$summaries[0].Groups[3].Value -ne 1 -or
        [int]$summaries[0].Groups[1].Value -ne [int](-not $negative) -or [int]$summaries[0].Groups[2].Value -ne [int]$negative) {
        throw 'Wrong selected-harness success/failure summary'
    }
    $verification = [regex]::Matches($Text, '(?m)^VERIFICATION:- (\S+)\s*$')
    $wanted = if ($negative) { 'FAILED' } else { 'SUCCESSFUL' }
    if ($verification.Count -ne 1 -or $verification[0].Groups[1].Value -cne $wanted) { throw 'Wrong verifier outcome' }
    $checks = [regex]::Matches($Text, '(?m)^\s*\*\* ([0-9]+) of ([1-9][0-9]*) failed(?: \(([0-9]+) unreachable\))?\s*$')
    if ($checks.Count -ne 1 -or [long]$checks[0].Groups[1].Value -gt [long]$checks[0].Groups[2].Value -or
        ($checks[0].Groups[3].Success -and [long]$checks[0].Groups[3].Value -ge [long]$checks[0].Groups[2].Value)) { throw 'Missing, malformed, or empty check summary' }
    $blocks = [regex]::Matches($Text, '(?ms)^Check [0-9]+: ([^\r\n]+)\r?\n(.*?)(?=^Check [0-9]+:|^SUMMARY:)')
    if ($blocks.Count -ne [long]$checks[0].Groups[2].Value) { throw 'Detailed check count does not match the summary total' }
    $failed = @($blocks | Where-Object { $_.Groups[2].Value -match '(?m)^\s*- Status: FAILURE\s*$' })
    if ($failed.Count -ne [long]$checks[0].Groups[1].Value) { throw 'Failed check count does not match detailed results' }
    if (-not $negative) {
        if ($failed.Count -ne 0) { throw 'Baseline contains a failed check' }
        return [ordered]@{ harness = $Harness; accepted = $true; failed_assertions = @() }
    }
    if ($failed.Count -eq 0) { throw 'Mutant has no failed assertion' }
    $names = foreach ($check in $failed) {
        $body = $check.Groups[2].Value
        if ($check.Groups[1].Value -notmatch '^properties::[A-Za-z_0-9]+\.assertion\.[0-9]+$' -or
            $body -notmatch '(?m)^\s*- Location: (?:[^\r\n]*[/\\])?src[/\\]bin[/\\]vocabulary[/\\]properties\.rs:[0-9]+:[0-9]+ in function properties::') {
            throw 'Unexpected failed check kind or source location (not the intended property assertion)'
        }
        $descriptions = [regex]::Matches($body, '(?m)^\s*- Description: ([^\r\n]+)$')
        if ($descriptions.Count -ne 1) { throw 'Missing failed assertion description' }
        $matches = @($Failures | Where-Object { $descriptions[0].Groups[1].Value -match ('(?<![A-Z_])' + [regex]::Escape($_) + '(?![A-Z_])') })
        if ($matches.Count -ne 1) { throw "Unexpected failed assertion: $($descriptions[0].Groups[1].Value)" }
        $matches[0]
    }
    return [ordered]@{ harness = $Harness; accepted = $true; failed_assertions = @($names) }
}

function Test-ControlParser {
    $good = "Checking harness proofs::control...`nCheck 1: properties::stop.assertion.1`n - Status: SUCCESS`n - Description: `"`"EXPECTED_PROPERTY`"`"`n - Location: src/bin/vocabulary/properties.rs:4:5 in function properties::stop`nSUMMARY:`n ** 0 of 1 failed`nVERIFICATION:- SUCCESSFUL`nComplete - 1 successfully verified harnesses, 0 failures, 1 total.`n"
    $bad = $good.Replace('Status: SUCCESS', 'Status: FAILURE').Replace('0 of 1 failed', '1 of 1 failed').Replace('VERIFICATION:- SUCCESSFUL', 'VERIFICATION:- FAILED').Replace('1 successfully verified harnesses, 0 failures', '0 successfully verified harnesses, 1 failures')
    $null = Assert-ControlLog $good 0 $false 'proofs::control'
    $null = Assert-ControlLog $bad 1 $false 'proofs::control' @('EXPECTED_PROPERTY')
    $cases = @(
        @{ text = $bad; code = 0; timeout = $false },
        @{ text = $bad; code = 2; timeout = $false },
        @{ text = $bad; code = 1; timeout = $true },
        @{ text = $bad; code = 124; timeout = $false },
        @{ text = ''; code = 1; timeout = $false },
        @{ text = $bad.Replace('EXPECTED_PROPERTY', 'OTHER_PROPERTY'); code = 1; timeout = $false },
        @{ text = $bad.Replace('properties.rs:4:5', 'model.rs:4:5'); code = 1; timeout = $false },
        @{ text = $bad.Replace('.assertion.1', '.unwind.1'); code = 1; timeout = $false },
        @{ text = $bad.Replace('proofs::control', 'proofs::other'); code = 1; timeout = $false },
        @{ text = $bad.Replace('1 failures, 1 total', '2 failures, 2 total'); code = 1; timeout = $false },
        @{ text = $bad.Replace('1 of 1 failed', '0 of 1 failed'); code = 1; timeout = $false },
        @{ text = $bad.Replace('1 of 1 failed', '1 of 2 failed'); code = 1; timeout = $false },
        @{ text = $bad.Replace('1 of 1 failed', '1 of 1 failed (1 unreachable)'); code = 1; timeout = $false },
        @{ text = $bad + "error[E0308]: compile failed`n"; code = 1; timeout = $false },
        @{ text = $bad + $bad; code = 1; timeout = $false }
    )
    foreach ($case in $cases) {
        $rejected = $false
        try { $null = Assert-ControlLog $case.text $case.code $case.timeout 'proofs::control' @('EXPECTED_PROPERTY') } catch { $rejected = $true }
        if (-not $rejected) { throw 'Control parser accepted a malformed negative fixture' }
    }
    Write-Output "Control parser: $($cases.Count + 2) fixtures passed (2 positive, $($cases.Count) rejection controls)."
}
if ($SelfTest) { Test-ControlParser; return }
if (-not $IsWindows) { throw 'This wrapper requires PowerShell 7 on Windows and an existing WSL installation.' }
if ($LinuxUser -notmatch '^[a-z_][a-z0-9_-]*$' -or $TargetDirectory -notmatch '^/[^\r\n]+$') { throw 'Invalid Linux user or absolute target directory' }
$LogRoot = [IO.Path]::GetFullPath($LogRoot)
if (Test-Path -LiteralPath $LogRoot) { throw 'Choose a new evidence directory; existing evidence is never overwritten.' }
$null = New-Item -ItemType Directory -Path $LogRoot
function Save-Json([string] $Path, $Value) { ConvertTo-Json -InputObject $Value -Depth 12 | Set-Content -LiteralPath $Path -Encoding utf8 }

function Inventory([string] $Root) {
    foreach ($package in @('i2c-one-shot-proofs', 'i2c-one-shot-core', 'i2c-one-shot-vocabulary', 'i2c-one-shot-interface')) {
        $directory = Join-Path $Root $package
        $files = @(Get-Item -LiteralPath (Join-Path $directory 'Cargo.toml'), (Join-Path $directory 'Cargo.lock'))
        $files += @(Get-ChildItem -LiteralPath (Join-Path $directory 'src') -Recurse -File)
        foreach ($file in $files | Sort-Object FullName) {
            [ordered]@{ path = [IO.Path]::GetRelativePath($Root, $file.FullName).Replace('\', '/'); sha256 = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash; bytes = $file.Length }
        }
    }
}
function Difference($Before, $After) {
    Compare-Object @($Before | ForEach-Object { "$($_.path)|$($_.sha256)|$($_.bytes)" }) @($After | ForEach-Object { "$($_.path)|$($_.sha256)|$($_.bytes)" })
}
function Linux-Path([string] $Path) {
    if ($Path -notmatch '^([A-Za-z]):[\\/](.*)$') { throw 'Source/evidence must reside on a Windows local drive.' }
    '/mnt/' + $Matches[1].ToLowerInvariant() + '/' + $Matches[2].Replace('\', '/')
}
$commands = [Collections.Generic.List[object]]::new()
function Capture([string] $Directory, [string] $Name, [string] $Executable, [string[]] $Arguments, [int] $Seconds) {
    $entry = [ordered]@{ name = $Name; executable = $Executable; arguments = $Arguments; started_utc = [DateTimeOffset]::UtcNow.ToString('o'); exit_code = $null; timed_out = $false }
    $commands.Add($entry)
    Save-Json (Join-Path $LogRoot 'commands.json') $commands
    $info = [Diagnostics.ProcessStartInfo]::new($Executable)
    $info.UseShellExecute = $false
    $info.CreateNoWindow = $true
    $info.RedirectStandardOutput = $true
    $info.RedirectStandardError = $true
    foreach ($argument in $Arguments) { $info.ArgumentList.Add($argument) }
    $process = [Diagnostics.Process]::new()
    $process.StartInfo = $info
    try {
        if (-not $process.Start()) { throw 'Could not launch owned child process' }
        $entry['pid'] = $process.Id
        $stdout = $process.StandardOutput.ReadToEndAsync()
        $stderr = $process.StandardError.ReadToEndAsync()
        if (-not $process.WaitForExit(($Seconds + 60) * 1000)) {
            $entry.timed_out = $true
            $process.Kill($true)
            if (-not $process.WaitForExit(15000)) { throw 'Owned child failed to exit after termination' }
        }
        $entry.exit_code = $process.ExitCode
        if ($entry.exit_code -in @(124, 137)) { $entry.timed_out = $true }
        if (-not [Threading.Tasks.Task]::WaitAll(@($stdout, $stderr), 15000)) { throw 'Could not collect owned child output' }
        $outText = $stdout.GetAwaiter().GetResult()
        $errText = $stderr.GetAwaiter().GetResult()
        [IO.File]::WriteAllText((Join-Path $Directory "$Name.stdout.log"), $outText)
        [IO.File]::WriteAllText((Join-Path $Directory "$Name.stderr.log"), $errText)
        return [ordered]@{ text = $outText + "`n" + $errText; exit_code = $entry.exit_code; timed_out = $entry.timed_out }
    } finally {
        $entry['finished_utc'] = [DateTimeOffset]::UtcNow.ToString('o')
        Save-Json (Join-Path $LogRoot 'commands.json') $commands
        $process.Dispose()
    }
}

$observe = @'
    fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        self.cpu.observe(cx, wake)
    }
'@
$cases = @(
    [ordered]@{ name = 'stop-without-current-fifo'; harness = 'proofs::stop_uses_current_fifo_state'; failures = @('STOP_CURRENT_NOT_READY', 'STOP_REQUIRES_FRESH_FIFO'); before = @'
        if !self.cpu.fifo_empty() {
            return Poll::Pending;
        }
        self.cpu.command(Command::Stop);
'@; after = @'
        self.cpu.command(Command::Stop);
'@ },
    [ordered]@{ name = 'observe-wrong-wake'; harness = 'proofs::observations_preserve_status_and_wake'; failures = @('OBSERVE_EXACT_CALL'); before = $observe; after = @'
    fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        let _ = wake;
        self.cpu.observe(cx, WakeOn::Completion)
    }
'@ },
    [ordered]@{ name = 'observe-drops-nack'; harness = 'proofs::observations_preserve_status_and_wake'; failures = @('OBSERVE_ALL_STATUS_BITS_UNCHANGED'); before = $observe; after = @'
    fn observe(&mut self, cx: &mut Context<'_>, wake: WakeOn) -> Status {
        let mut status = self.cpu.observe(cx, wake);
        status.nack = false;
        status
    }
'@ },
    [ordered]@{ name = 'receive-truncates-buffer'; harness = 'proofs::rx_poll_preserves_owner_and_outcome'; failures = @('RX_EXACT_CONTENTS_AND_LENGTH'); before = '.map(core::convert::identity, |(cpu, bytes)| (Self::new(cpu), bytes))'; after = @'
.map(core::convert::identity, |(cpu, bytes)| {
                let length = bytes.len() - 1;
                (Self::new(cpu), &mut bytes[..length])
            })
'@ }
)
$linuxHome = "/home/$LinuxUser"
$prefix = @('--distribution', $Distribution, '--user', $LinuxUser, '--cd', $linuxHome, '--exec', '/usr/bin/env', '-u', 'CARGO_ENCODED_RUSTFLAGS', '-u', 'RUSTFLAGS', '-u', 'RUSTDOCFLAGS', '-u', 'CARGO_BUILD_TARGET', '-u', 'CARGO_TARGET_DIR', "PATH=$linuxHome/.cargo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin", 'RUSTUP_TOOLCHAIN=stable', '/usr/bin/timeout', '--signal=TERM', '--kill-after=15s', "${TimeoutSeconds}s", "$linuxHome/.cargo/bin/cargo", 'kani')
$sourceRoot = Split-Path -Parent $PSScriptRoot
$before = @(Inventory $sourceRoot)
Save-Json (Join-Path $LogRoot 'source-before.json') $before
Save-Json (Join-Path $LogRoot 'controls.json') $cases
$result = [ordered]@{ accepted = $false; controls = @(); failure = $null; source_unchanged = $false; scope = 'Four selected negative controls, not the full 3+9 proof suite.' }
$mutex = [Threading.Mutex]::new($false, 'Local\I2cOneShotKaniControlRunner')
$owned = $false
try {
    try { $owned = $mutex.WaitOne(0) } catch [Threading.AbandonedMutexException] { $owned = $true; throw 'Prior runner terminated unexpectedly; inspect its evidence/processes first.' }
    if (-not $owned) { throw 'Another proof runner is active.' }
    foreach ($case in $cases) {
        $directory = Join-Path $LogRoot $case.name
        $fixture = Join-Path $directory 'fixture'
        $null = New-Item -ItemType Directory -Path $fixture
        foreach ($file in $before) {
            $destination = Join-Path $fixture $file.path
            $null = [IO.Directory]::CreateDirectory((Split-Path -Parent $destination))
            Copy-Item -LiteralPath (Join-Path $sourceRoot $file.path) -Destination $destination
        }
        $baseline = @(Inventory $fixture)
        $copyDifferences = @(Difference $before $baseline)
        Save-Json (Join-Path $directory 'baseline-source.json') $baseline
        Save-Json (Join-Path $directory 'copy-differences.json') $copyDifferences
        if ($copyDifferences.Count -ne 0) { throw "Fixture is not byte-identical: $($case.name)" }
        $manifest = Linux-Path (Join-Path $fixture 'i2c-one-shot-proofs/Cargo.toml')
        $arguments = $prefix + @('--manifest-path', $manifest, '--target-dir', $TargetDirectory, '--output-format', 'regular', '--bin', 'vocabulary', '--harness', $case.harness, '--exact')
        $baselineLog = Capture $directory 'baseline' 'wsl.exe' $arguments $TimeoutSeconds
        $baselineProof = Assert-ControlLog $baselineLog.text $baselineLog.exit_code $baselineLog.timed_out $case.harness
        if (@(Difference $baseline @(Inventory $fixture)).Count -ne 0) { throw 'Baseline verification changed source or a lockfile.' }
        Write-Output "$($case.name): baseline passed."
        $path = Join-Path $fixture 'i2c-one-shot-vocabulary/src/vocabulary.rs'
        $original = [IO.File]::ReadAllText($path)
        $newline = if ($original.Contains("`r`n")) { "`r`n" } else { "`n" }
        $needle = $case.before.Replace("`r`n", "`n").Replace("`n", $newline)
        $replacement = $case.after.Replace("`r`n", "`n").Replace("`n", $newline)
        if ([regex]::Matches($original, [regex]::Escape($needle)).Count -ne 1) { throw "Mutation anchor is not unique: $($case.name)" }
        $backup = Join-Path $directory 'vocabulary.baseline.rs'
        Copy-Item -LiteralPath $path -Destination $backup
        [IO.File]::WriteAllText($path, $original.Replace($needle, $replacement), [Text.UTF8Encoding]::new($false))
        $mutant = @(Inventory $fixture)
        $expectedMutant = @($baseline | ForEach-Object { if ($_.path -eq 'i2c-one-shot-vocabulary/src/vocabulary.rs') { $mutant | Where-Object { $_.path -eq 'i2c-one-shot-vocabulary/src/vocabulary.rs' } } else { $_ } })
        if (@(Difference $expectedMutant $mutant).Count -ne 0) { throw 'Mutation changed unexpected files.' }
        Save-Json (Join-Path $directory 'mutant-source.json') $mutant
        $diff = Capture $directory 'mutation-diff' 'git.exe' @('diff', '--no-index', '--no-ext-diff', '--', $backup, $path) 30
        if ($diff.exit_code -ne 1 -or $diff.timed_out) { throw 'Could not capture the exact mutation diff.' }
        $negativeLog = Capture $directory 'mutant' 'wsl.exe' $arguments $TimeoutSeconds
        $negativeProof = Assert-ControlLog $negativeLog.text $negativeLog.exit_code $negativeLog.timed_out $case.harness $case.failures
        $afterMutant = @(Inventory $fixture)
        Save-Json (Join-Path $directory 'after-source.json') $afterMutant
        if (@(Difference $mutant $afterMutant).Count -ne 0) { throw 'Mutant verification changed source or a lockfile.' }
        $record = [ordered]@{ name = $case.name; baseline = $baselineProof; mutant = $negativeProof; source_unchanged_during_runs = $true; lockfiles_unchanged = $true }
        Save-Json (Join-Path $directory 'result.json') $record
        $result.controls += $record
        Save-Json (Join-Path $LogRoot 'result.json') $result
        Write-Output "$($case.name): intended assertion rejected the mutant."
    }
} catch { $result.failure = $_.Exception.Message } finally {
    try {
        $after = @(Inventory $sourceRoot)
        Save-Json (Join-Path $LogRoot 'source-after.json') $after
        $differences = @(Difference $before $after)
        Save-Json (Join-Path $LogRoot 'source-differences.json') $differences
        $result.source_unchanged = $differences.Count -eq 0
        if (-not $result.source_unchanged) { $result.failure = 'Checkout source changed during the controls; results are not accepted.' }
    } catch { $result.failure = "Final source check failed: $($_.Exception.Message)" }
    $result.accepted = $null -eq $result.failure -and $result.controls.Count -eq 4 -and $result.source_unchanged
    Save-Json (Join-Path $LogRoot 'result.json') $result
    if ($owned) { $mutex.ReleaseMutex() }
    $mutex.Dispose()
}
if (-not $result.accepted) { throw "Negative controls rejected: $($result.failure). Evidence: $LogRoot" }
Write-Output "All four independent baseline/mutant controls passed. Evidence retained: $LogRoot"

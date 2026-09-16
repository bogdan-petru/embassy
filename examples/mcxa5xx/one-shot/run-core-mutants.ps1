#requires -Version 7.0
<#
.SYNOPSIS
Run native protocol mutations in a small, preserved source snapshot.
.DESCRIPTION
Never mutates the checkout. Only protocol.rs is mutated, and only the protocol
and properties integration tests run. This is not the named safety-mutation
runner, an MMIO test, Miri, or a hardware test. See CORE-MUTATIONS.md.
#>
[CmdletBinding()]
param(
    [string]$OutputRoot,
    [switch]$ListOnly,
    [string]$Match,
    [ValidateRange(1, 100000)][int]$Cases = 256,
    [UInt64]$Seed = 20260914,
    [ValidateRange(1, 3600)][int]$TestTimeoutSeconds = 60,
    [ValidateRange(1, 3600)][int]$BuildTimeoutSeconds = 180
)

$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
$experimental = Join-Path $repo 'embassy-mcxa/experimental'
$toolchain = '1.92'
$hostTarget = 'x86_64-pc-windows-msvc'
$requiredTool = 'cargo-mutants 27.1.0'
$toolVersion = (& cargo "+$toolchain" mutants --version 2>&1 | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or $toolVersion -ne $requiredTool) {
    throw "Expected $requiredTool. Install with: cargo +1.92 install --locked cargo-mutants --version 27.1.0"
}

if (!$OutputRoot) {
    $runName = 'core-mutants-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0, 8)
    $OutputRoot = Join-Path $repo "examples/mcxa5xx/target/$runName"
}
$OutputRoot = [IO.Path]::GetFullPath($OutputRoot)
if (Test-Path -LiteralPath $OutputRoot) {
    throw 'OutputRoot must not exist; each campaign keeps its own source, lockfile, logs, and results.'
}
$volume = [IO.DriveInfo]::new([IO.Path]::GetPathRoot($OutputRoot))
if (!$ListOnly -and $volume.AvailableFreeSpace -lt 2GB) {
    throw 'Less than 2 GiB free on the output volume. No caches are deleted by this script.'
}

# Deliberate inventory, not a recursive copy of the repository or its caches.
# Regression inputs accompany the tests; mutant-discovered failures are logged
# but are not persisted back into this immutable input set during a campaign.
$files = [Collections.Generic.List[string]]::new()
foreach ($crate in @('i2c-one-shot-core', 'i2c-one-shot-vocabulary', 'i2c-one-shot-interface')) {
    $files.Add("$crate/Cargo.toml")
    $source = Join-Path $experimental "$crate/src"
    foreach ($file in Get-ChildItem -LiteralPath $source -File -Recurse) {
        $files.Add([IO.Path]::GetRelativePath($experimental, $file.FullName))
    }
}
foreach ($relative in @('Cargo.lock', 'tests/protocol.rs', 'tests/properties.rs')) {
    $files.Add("i2c-one-shot-core/$relative")
}
$core = Join-Path $experimental 'i2c-one-shot-core'
foreach ($file in Get-ChildItem -LiteralPath (Join-Path $core 'tests') -File -Filter '*.proptest-regressions' -Recurse) {
    $files.Add([IO.Path]::GetRelativePath($experimental, $file.FullName))
}
$regressions = Join-Path $core 'proptest-regressions'
if (Test-Path -LiteralPath $regressions) {
    foreach ($file in Get-ChildItem -LiteralPath $regressions -File -Recurse) {
        $files.Add([IO.Path]::GetRelativePath($experimental, $file.FullName))
    }
}
foreach ($relative in $files) {
    $item = Get-Item -LiteralPath (Join-Path $experimental $relative)
    if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) {
        throw "Snapshot inputs must be ordinary files: $relative"
    }
}

$snapshot = Join-Path $OutputRoot 'source'
$snapshotCore = Join-Path $snapshot 'i2c-one-shot-core'
$inventory = foreach ($relative in ($files | Sort-Object -Unique)) {
    $source = Join-Path $experimental $relative
    $destination = Join-Path $snapshot $relative
    New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($destination)) -Force | Out-Null
    Copy-Item -LiteralPath $source -Destination $destination
    [ordered]@{ path = $relative.Replace('\', '/'); sha256 = (Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash }
}
$inventory | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $OutputRoot 'input-files.json') -Encoding utf8
$testRunner = Join-Path $OutputRoot 'mutation-test-runner.ps1'
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'mutation-test-runner.ps1') -Destination $testRunner
# Cargo's runner environment string splits whitespace rather than shell quotes.
# An argument array in this snapshot-only config also works for paths with spaces.
$runnerConfig = Join-Path $snapshotCore '.cargo/config.toml'
New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($runnerConfig)) -Force | Out-Null
@'
[target.x86_64-pc-windows-msvc]
runner = ["pwsh", "-NoProfile", "-File", "../../mutation-test-runner.ps1"]
'@ | Set-Content -LiteralPath $runnerConfig -Encoding utf8

$argsForMutants = @(
    "+$toolchain", 'mutants', '--dir', $snapshotCore, '--no-config',
    '--file=src/protocol.rs', '--package=i2c-one-shot-core',
    '--error=crate::Error::Dma', '--no-shuffle', '--in-place',
    '--baseline=run', '--cap-lints=true', '--colors=never', '--annotations=none',
    "--timeout=$TestTimeoutSeconds", "--build-timeout=$BuildTimeoutSeconds",
    '--cargo-arg=--locked', "--cargo-arg=--target=$hostTarget",
    '--cargo-arg=--test=protocol', '--cargo-arg=--test=properties',
    '--output', $OutputRoot, '--caught', '--unviable'
)
if ($Match) { $argsForMutants += @('--re', $Match) }
$savedEnvironment = @{}
$environmentNames = @(
    'RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'CARGO_BUILD_TARGET', 'CARGO_TARGET_DIR',
    'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'CARGO_INCREMENTAL', 'CARGO_MUTANTS_JOBS',
    'CARGO_TARGET_X86_64_PC_WINDOWS_MSVC_RUNNER',
    'PROPTEST_CASES', 'PROPTEST_RNG_SEED', 'PROPTEST_RNG_ALGORITHM', 'PROPTEST_DISABLE_FAILURE_PERSISTENCE'
    (Get-ChildItem Env:PROPTEST_* | Select-Object -ExpandProperty Name)
) | Sort-Object -Unique
foreach ($name in $environmentNames) {
    $savedEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
    Remove-Item -LiteralPath "Env:$name" -ErrorAction SilentlyContinue
}
$campaignExit = 1
try {
    $env:CARGO_BUILD_TARGET = $hostTarget
    $env:CARGO_TARGET_DIR = Join-Path $OutputRoot 'build'
    $env:CARGO_INCREMENTAL = '0'
    $env:PROPTEST_CASES = "$Cases"
    $env:PROPTEST_RNG_SEED = "$Seed"
    $env:PROPTEST_RNG_ALGORITHM = 'cc'
    $env:PROPTEST_DISABLE_FAILURE_PERSISTENCE = '1'
    $metadata = [ordered]@{
        created_utc = [DateTime]::UtcNow.ToString('o')
        source_head = (& git -C $repo rev-parse HEAD).Trim()
        source_is_snapshot_of_current_files = $true
        rustc = (& rustc "+$toolchain" -Vv | Out-String).Trim()
        tool = $toolVersion
        runner_sha256 = (Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash
        cargo_arguments = $argsForMutants
        environment = [ordered]@{
            PROPTEST_CASES = $env:PROPTEST_CASES
            PROPTEST_RNG_SEED = $env:PROPTEST_RNG_SEED
            PROPTEST_RNG_ALGORITHM = $env:PROPTEST_RNG_ALGORITHM
            PROPTEST_DISABLE_FAILURE_PERSISTENCE = $env:PROPTEST_DISABLE_FAILURE_PERSISTENCE
            CARGO_INCREMENTAL = $env:CARGO_INCREMENTAL
        }
        test_runner_sha256 = (Get-FileHash -LiteralPath $testRunner -Algorithm SHA256).Hash
        runner_config = Get-Content -LiteralPath $runnerConfig -Raw
        free_bytes_before = $volume.AvailableFreeSpace
    }
    $metadata | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $OutputRoot 'run.json') -Encoding utf8

    # List first so an empty selector is a failure, not a successful zero-test run.
    $candidateText = & cargo @argsForMutants --list --json 2> (Join-Path $OutputRoot 'list-stderr.log')
    $listExit = $LASTEXITCODE
    $candidateText | Set-Content -LiteralPath (Join-Path $OutputRoot 'candidates.json') -Encoding utf8
    if ($listExit -ne 0) { throw "Mutant enumeration failed (exit $listExit)." }
    $candidates = @($candidateText | ConvertFrom-Json)
    if ($candidates.Count -eq 0) { throw 'No mutants selected; refusing an empty campaign.' }
    Write-Host "Selected $($candidates.Count) mutants. Evidence: $OutputRoot"
    if ($ListOnly) {
        $campaignExit = 0
    } else {
        & cargo @argsForMutants 2>&1 | Tee-Object -FilePath (Join-Path $OutputRoot 'console.log')
        $campaignExit = $LASTEXITCODE
        $outcomePath = Join-Path $OutputRoot 'mutants.out/outcomes.json'
        if (!(Test-Path -LiteralPath $outcomePath)) { throw 'Mutation outcomes are missing.' }
        $outcomes = Get-Content -LiteralPath $outcomePath -Raw | ConvertFrom-Json
        $baseline = @($outcomes.outcomes | Where-Object { $_.scenario -is [string] -and $_.scenario -eq 'Baseline' })
        $mutantResults = @($outcomes.outcomes | Where-Object { $_.scenario -isnot [string] })
        $actualNames = @($mutantResults | ForEach-Object { $_.scenario.Mutant.name })
        $expectedNames = @($candidates | ForEach-Object { $_.name })
        if ($baseline.Count -ne 1 -or $baseline[0].summary -ne 'Success' -or
            @($baseline[0].phase_results | Where-Object { $_.phase -eq 'Build' -and $_.process_status -eq 'Success' }).Count -ne 1 -or
            @($baseline[0].phase_results | Where-Object { $_.phase -eq 'Test' -and $_.process_status -eq 'Success' }).Count -ne 1 -or
            $mutantResults.Count -ne $candidates.Count -or
            @($actualNames | Sort-Object -Unique).Count -ne $candidates.Count -or
            @(Compare-Object $expectedNames $actualNames).Count -ne 0) {
            throw 'Baseline or per-candidate accounting is incomplete; this is not a qualified campaign.'
        }
        if (@($mutantResults | Where-Object summary -eq 'MissedMutant').Count -ne 0) {
            $campaignExit = 2
        }
        # A linker/tool failure is not a compiler rejection of an opaque owner.
        # Only the two reviewed default-constructor diagnostics qualify here.
        # Likewise, an abnormal process exit alone cannot count as a killed bug.
        $unqualified = @(
            foreach ($outcome in $outcomes.outcomes) {
                $log = Get-Content -LiteralPath (Join-Path $OutputRoot "mutants.out/$($outcome.log_path)") -Raw
                if ($log -match 'error: linking|fatal error LNK|internal compiler error') {
                    $outcome.log_path
                } elseif ($outcome.summary -eq 'Unviable') {
                    $codes = @([regex]::Matches($log, 'error\[(E\d{4})\]') | ForEach-Object { $_.Groups[1].Value })
                    if ($codes.Count -eq 0 -or @($codes | Where-Object { $_ -notin @('E0277', 'E0599') }).Count -ne 0) {
                        $outcome.log_path
                    }
                } elseif ($outcome.summary -eq 'CaughtMutant' -and
                    ($log -notmatch 'test result: FAILED\.' -or $log -notmatch 'panicked at tests[\\/](properties|protocol)\.rs' -or
                     @($outcome.phase_results | Where-Object {
                         $_.phase -eq 'Test' -and $_.process_status -isnot [string] -and
                         $_.process_status.PSObject.Properties.Name -contains 'Failure' -and $_.process_status.Failure -ne 0
                     }).Count -ne 1)) {
                    $outcome.log_path
                } elseif ($outcome.summary -eq 'Success' -and $outcome.scenario -isnot [string]) {
                    $outcome.log_path
                } elseif ($outcome.summary -notin @('Success', 'CaughtMutant', 'MissedMutant', 'Unviable')) {
                    $outcome.log_path
                }
            }
        )
        ConvertTo-Json -InputObject $unqualified | Set-Content -LiteralPath (Join-Path $OutputRoot 'unqualified-outcomes.json') -Encoding utf8
        if ($unqualified.Count -ne 0) {
            Write-Warning "$($unqualified.Count) outcomes need infrastructure/compiler review; this is not a passing campaign."
            $campaignExit = 3
        }
    }
    # Cargo-mutants restores its edits; verify the preserved input source agrees.
    foreach ($entry in $inventory) {
        if ((Get-FileHash -LiteralPath (Join-Path $snapshot $entry.path) -Algorithm SHA256).Hash -ne $entry.sha256) {
            throw "Snapshot input changed during campaign: $($entry.path). Preserve it for diagnosis."
        }
    }
    [ordered]@{
        exit_code = $campaignExit
        list_only = [bool]$ListOnly
        selected_mutants = $candidates.Count
        free_bytes_after = ([IO.DriveInfo]::new($volume.Name)).AvailableFreeSpace
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $OutputRoot 'result.json') -Encoding utf8
} finally {
    foreach ($name in $environmentNames) {
        Remove-Item -LiteralPath "Env:$name" -ErrorAction SilentlyContinue
        if ($null -ne $savedEnvironment[$name]) {
            [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name], 'Process')
        }
    }
}
Write-Host "Finished with exit $campaignExit. No checkout files were mutated; all evidence is preserved at $OutputRoot."
exit $campaignExit

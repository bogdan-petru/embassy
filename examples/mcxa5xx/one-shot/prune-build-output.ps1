<#
.SYNOPSIS
Prune Cargo build output from the three live target trees while keeping every
hardware, mutation and review evidence directory, and preserving the exact
firmware binaries the checkpoint records cite by SHA-256.

.DESCRIPTION
Operates on:
  <repo>/target
  <repo>/embassy-mcxa/target
  <repo>/examples/mcxa5xx/target

Deletes ONLY entries whose names are known Cargo output (an allowlist):
  directories: debug, release, doc, tmp, and target triples such as
               thumbv8m.main-none-eabihf or x86_64-pc-windows-msvc
  files:       .rustc_info.json, .rustdoc_fingerprint.json, CACHEDIR.TAG

Everything else at the top of each tree is kept by construction. That includes
the evidence directories the checkpoint records cite (i2c-one-shot-hil,
-rxstall, -safety, -watchdog, i2c-teaching-hil, reset-recovery-hil,
review-twoboard, reset-mutation-*, reset-validation, full-driver-safeguards,
review-mcxa2/5, one-shot-mutations-*) and the reset-sequence control binaries.

Recorded firmware: the records in examples/mcxa5xx/one-shot/*.md identify
flashed firmware by SHA-256 only, while the ELF files live under
<triple>/release. Before a triple directory is deleted, every extensionless
binary in its release/ is hashed; any whose hash appears in the records is
copied to examples/mcxa5xx/target/preserved-firmware/<name>-<sha12>.elf and
the copy is re-hashed. A copy that does not verify aborts before deletion.

Fail-closed rules:
  * Dry run by default. Nothing is deleted or copied without -Execute.
  * Refuses if the repository layout is not where this script expects it.
  * Refuses while cargo, rustc, rustdoc or rust-analyzer is running, because a
    locked file would leave a half-pruned tree (override: -IgnoreRunningTools).
  * Refuses if any deletion candidate contains an evidence marker
    (verdict.json, ledger.md, *.out.log, *.err.log) within three levels.
  * A locked or undeletable item is reported and the script exits non-zero;
    it never silently skips.

.PARAMETER Execute
Actually preserve and delete. Without it the script only reports.

.PARAMETER KeepFirmware
Do not delete target-triple directories at all (no preservation needed).

.PARAMETER IgnoreRunningTools
Proceed even if cargo/rustc/rustdoc/rust-analyzer processes are running.

.EXAMPLE
./one-shot/prune-build-output.ps1                 # dry run: sizes, kept paths, recorded ELFs
./one-shot/prune-build-output.ps1 -Execute        # preserve recorded ELFs, then prune
./one-shot/prune-build-output.ps1 -Execute -KeepFirmware
#>
[CmdletBinding()]
param(
    [switch]$Execute,
    [switch]$KeepFirmware,
    [switch]$IgnoreRunningTools
)

$ErrorActionPreference = 'Stop'

# ---- PowerShell 7 is required, and not for style: Windows PowerShell 5.1's
# Remove-Item -Recurse fails on paths longer than 260 characters, which Cargo's
# deps/ and build/ trees exceed under this repository. A failure mid-tree would
# leave a half-pruned target directory. pwsh (PowerShell 7) handles long paths.
if ($PSVersionTable.PSVersion.Major -lt 7) {
    Write-Host "This script needs PowerShell 7 (pwsh); you are running Windows PowerShell $($PSVersionTable.PSVersion)." -ForegroundColor Red
    Write-Host "Reason: Windows PowerShell cannot reliably delete Cargo's long build paths." -ForegroundColor Red
    Write-Host ""
    Write-Host "Run it as:" -ForegroundColor Cyan
    Write-Host "    pwsh $($MyInvocation.MyCommand.Path)" -ForegroundColor Cyan
    Write-Host "or open a PowerShell 7 terminal (the other one-shot runners require it too)." -ForegroundColor Cyan
    exit 1
}

# ---- repository layout (resolved from this file, not the current directory) ----
$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..')).Path
foreach ($required in 'embassy-mcxa\Cargo.toml', 'examples\mcxa5xx\Cargo.toml', 'embassy-mcxa\src\i2c\one_shot') {
    if (-not (Test-Path -LiteralPath (Join-Path $root $required))) {
        throw "Unexpected repository layout: '$required' not found under $root"
    }
}
$trees       = @('target', 'embassy-mcxa\target', 'examples\mcxa5xx\target') | ForEach-Object { Join-Path $root $_ }
$recordsGlob = Join-Path $root 'examples\mcxa5xx\one-shot\*.md'
$preserveDir = Join-Path $root 'examples\mcxa5xx\target\preserved-firmware'

# ---- allowlist of Cargo output names ----
$allowDirs       = @('debug', 'release', 'doc', 'tmp')
$triplePattern   = '^(thumbv[0-9a-z.\-]+|x86_64-[a-z0-9_\-]+|aarch64-[a-z0-9_\-]+|riscv[0-9a-z_\-]+|wasm32-[a-z0-9_\-]+)$'
$allowFiles      = @('.rustc_info.json', '.rustdoc_fingerprint.json', 'CACHEDIR.TAG')
$evidenceMarkers = @('verdict.json', 'ledger.md', '*.out.log', '*.err.log')

# ---- running toolchain processes would leave locked files behind ----
if (-not $IgnoreRunningTools) {
    $busy = Get-Process -Name cargo, rustc, rustdoc, rust-analyzer -ErrorAction SilentlyContinue
    if ($busy) {
        throw "Refusing: toolchain processes are running ($(($busy.Name | Sort-Object -Unique) -join ', ')). Stop them or pass -IgnoreRunningTools."
    }
}

# ---- SHA-256 values the records cite ----
$recordedHashes = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
foreach ($doc in Get-ChildItem -Path $recordsGlob -File) {
    foreach ($m in [regex]::Matches((Get-Content -LiteralPath $doc.FullName -Raw), '\b[0-9A-Fa-f]{64}\b')) {
        [void]$recordedHashes.Add($m.Value)
    }
}

function Get-TreeSize([string]$Path) {
    $sum = 0L
    Get-ChildItem -LiteralPath $Path -Recurse -Force -File -ErrorAction SilentlyContinue |
        ForEach-Object { $sum += $_.Length }
    return $sum
}

function Get-RecordedBinaries([string]$TripleDir) {
    # Extensionless top-level files in release/ are the built binaries.
    $rel = Join-Path $TripleDir 'release'
    if (-not (Test-Path -LiteralPath $rel)) { return @() }
    $hits = @()
    $others = 0
    foreach ($f in Get-ChildItem -LiteralPath $rel -File -ErrorAction SilentlyContinue | Where-Object { -not $_.Extension -and $_.Name -notmatch '^\.' }) {
        $h = (Get-FileHash -LiteralPath $f.FullName -Algorithm SHA256).Hash
        if ($recordedHashes.Contains($h)) { $hits += [pscustomobject]@{ Name = $f.Name; Hash = $h; Path = $f.FullName } }
        else { $others++ }
    }
    return [pscustomobject]@{ Recorded = $hits; Others = $others }
}

# ---- build the plan ----
$plan = [System.Collections.Generic.List[object]]::new()
$kept = [System.Collections.Generic.List[string]]::new()
foreach ($tree in $trees) {
    if (-not (Test-Path -LiteralPath $tree)) { continue }
    foreach ($entry in Get-ChildItem -LiteralPath $tree -Force) {
        $isTriple = $entry.PSIsContainer -and ($entry.Name -match $triplePattern)
        $delete =
            ($entry.PSIsContainer -and ($allowDirs -contains $entry.Name)) -or
            ($isTriple -and -not $KeepFirmware) -or
            (-not $entry.PSIsContainer -and ($allowFiles -contains $entry.Name))

        if (-not $delete) { $kept.Add($entry.FullName); continue }

        if ($entry.PSIsContainer) {
            # Evidence never lives inside Cargo output. If it does, stop.
            $marker = Get-ChildItem -LiteralPath $entry.FullName -Recurse -Depth 3 -Force -File -Include $evidenceMarkers -ErrorAction SilentlyContinue |
                Select-Object -First 1
            if ($marker) {
                throw "Refusing: evidence marker found inside a build directory: $($marker.FullName)"
            }
        }

        $bins = if ($isTriple) { Get-RecordedBinaries $entry.FullName } else { $null }

        Write-Host "measuring $($entry.FullName) ..." -ForegroundColor DarkGray
        $plan.Add([pscustomobject]@{
            Path     = $entry.FullName
            IsDir    = $entry.PSIsContainer
            Bytes    = if ($entry.PSIsContainer) { Get-TreeSize $entry.FullName } else { $entry.Length }
            Recorded = if ($bins) { @($bins.Recorded) } else { @() }
            Others   = if ($bins) { $bins.Others } else { 0 }
        })
    }
}

# ---- report ----
$rel = { param($p) $p.Substring($root.Length + 1) }
$total = ($plan | Measure-Object -Property Bytes -Sum).Sum
Write-Host ''
Write-Host ("{0,10}  {1}" -f 'SIZE', 'WILL DELETE')
foreach ($item in $plan | Sort-Object Bytes -Descending) {
    Write-Host ("{0,10}  {1}" -f ('{0:N1} GiB' -f ($item.Bytes / 1GB)), (& $rel $item.Path))
    foreach ($b in $item.Recorded) {
        Write-Host ("{0,10}    preserve recorded firmware: {1}  {2}..." -f '', $b.Name, $b.Hash.Substring(0, 12)) -ForegroundColor Yellow
    }
    if ($item.Others -gt 0) {
        Write-Host ("{0,10}    {1} other binaries not cited by any record: deleted" -f '', $item.Others) -ForegroundColor DarkGray
    }
}
Write-Host ("{0,10}  TOTAL" -f ('{0:N1} GiB' -f ($total / 1GB)))
Write-Host ''
Write-Host "Recorded SHA-256 values found in one-shot/*.md: $($recordedHashes.Count)"
Write-Host ''
Write-Host 'KEPT (not on the Cargo-output allowlist):'
foreach ($k in $kept) { Write-Host ("            {0}" -f (& $rel $k)) }
Write-Host ''

if (-not $Execute) {
    Write-Host 'Dry run. Re-run with -Execute to preserve recorded firmware and delete the items above.' -ForegroundColor Cyan
    exit 0
}

# ---- preserve recorded firmware before any deletion; verify every copy ----
$toPreserve = $plan | ForEach-Object { $_.Recorded } | Where-Object { $_ }
if ($toPreserve) {
    New-Item -ItemType Directory -Path $preserveDir -Force | Out-Null
    foreach ($b in $toPreserve) {
        $dest = Join-Path $preserveDir ("{0}-{1}.elf" -f $b.Name, $b.Hash.Substring(0, 12).ToLower())
        Copy-Item -LiteralPath $b.Path -Destination $dest -Force
        $check = (Get-FileHash -LiteralPath $dest -Algorithm SHA256).Hash
        if ($check -ne $b.Hash) {
            throw "Refusing to delete: preserved copy hash mismatch for $($b.Name) ($check != $($b.Hash)). Nothing was deleted."
        }
        Write-Host ("preserved {0} -> {1}" -f $b.Name, (& $rel $dest))
    }
}

# ---- delete, reporting every failure, never skipping silently ----
$failures = [System.Collections.Generic.List[string]]::new()
foreach ($item in $plan) {
    try {
        Remove-Item -LiteralPath $item.Path -Recurse -Force -Confirm:$false -ErrorAction Stop
        Write-Host ("deleted  {0}" -f (& $rel $item.Path))
    } catch {
        $failures.Add("$($item.Path): $($_.Exception.Message)")
        Write-Host ("FAILED   {0}: {1}" -f (& $rel $item.Path), $_.Exception.Message) -ForegroundColor Red
    }
}

$free = (Get-PSDrive -Name ($root.Substring(0, 1))).Free
Write-Host ''
Write-Host ("Free space on {0}: {1:N1} GiB" -f $root.Substring(0, 2), ($free / 1GB))
if ($failures.Count -gt 0) {
    Write-Host ("{0} item(s) could not be deleted (locked or in use). Nothing else was skipped." -f $failures.Count) -ForegroundColor Red
    exit 1
}
Write-Host 'Prune complete; every kept path above is untouched.' -ForegroundColor Green
exit 0

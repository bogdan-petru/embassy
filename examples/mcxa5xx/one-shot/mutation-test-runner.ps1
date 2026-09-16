#requires -Version 7.0
# A native Cargo target runner used only by run-core-mutants.ps1. Relinking a
# just-executed EXE produced LNK1104 on this Windows host. Run a byte-identical,
# uniquely named copy instead, leaving Cargo's original available for relinking.
# Preserve copies as evidence; do not retry a failure or change its exit code.
param(
    [Parameter(Mandatory, Position = 0)][string]$Executable,
    [Parameter(ValueFromRemainingArguments)][string[]]$TestArguments
)
$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
# Keep the copy beside the original for identical local-DLL lookup semantics.
$executionDirectory = [IO.Path]::GetDirectoryName([IO.Path]::GetFullPath($Executable))
$copy = Join-Path $executionDirectory ([IO.Path]::GetFileNameWithoutExtension($Executable) + '-' + [guid]::NewGuid().ToString('N') + '.exe')
Copy-Item -LiteralPath $Executable -Destination $copy
$originalHash = (Get-FileHash -LiteralPath $Executable -Algorithm SHA256).Hash
if ((Get-FileHash -LiteralPath $copy -Algorithm SHA256).Hash -ne $originalHash) {
    throw 'Test executable copy does not match the original.'
}
Write-Output "MUTATION_TEST_COPY sha256=$originalHash path=$copy"
& $copy @TestArguments
$testExit = $LASTEXITCODE
exit $testExit

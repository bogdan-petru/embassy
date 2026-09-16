# Hardware-free runner sequencing controls.  Extract named functions from the
# runner rather than dot-sourcing it: its top-level body deliberately flashes
# physical boards.
$ErrorActionPreference = 'Stop'
$passed = 0

function Check([bool]$Condition, [string]$Name) {
    if (-not $Condition) { throw "FAILED: $Name" }
    $script:passed++
}

function Reject([scriptblock]$Action, [string]$Name) {
    $rejected = $false
    try { & $Action } catch { $rejected = $true }
    Check $rejected $Name
}

function RejectWithMessage([scriptblock]$Action, [string]$Expected, [string]$Name) {
    $message = $null
    try { & $Action } catch { $message = $_.Exception.Message }
    Check ($null -ne $message -and $message.Contains($Expected)) $Name
}

$runner = Join-Path $PSScriptRoot '..\run-rxstall.ps1'
$verdicts = Join-Path $PSScriptRoot '..\rxstall-verdicts.ps1'
. $verdicts
$tokens = $null
$parseErrors = $null
$script:runnerAst = [System.Management.Automation.Language.Parser]::ParseFile(
    $runner, [ref]$tokens, [ref]$parseErrors)
if ($parseErrors.Count -ne 0) {
    throw "Cannot parse RXSTALL runner: $($parseErrors[0].Message)"
}

function Import-RunnerFunction([string]$Name) {
    $definitions = @($script:runnerAst.FindAll({
        param($node)
        $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq $Name
    }, $true))
    if ($definitions.Count -ne 1) { throw "Expected exactly one runner function named $Name." }
    # Installing the body explicitly in script scope keeps it available after
    # this loader returns, while still evaluating no runner top-level code.
    $body = $definitions[0].Body.Extent.Text
    $parameters = [string]::Join(', ', @($definitions[0].Parameters | ForEach-Object { $_.Extent.Text }))
    $functionBody = "param($parameters)`n" + $body.Substring(1, $body.Length - 2)
    Set-Item -Path "Function:script:$Name" -Value ([scriptblock]::Create($functionBody))
}

function New-FakeProcess {
    $process = [pscustomobject]@{ HasExited = $true; ExitCode = 0 }
    $process | Add-Member -MemberType ScriptMethod -Name WaitForExit -Value { }
    $process | Add-Member -MemberType ScriptMethod -Name Dispose -Value { }
    return $process
}

$testParent = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath()).TrimEnd('\\')
$testLeaf = 'rxstall-startup-' + [guid]::NewGuid().ToString('N')
$testRoot = Join-Path $testParent $testLeaf
try {
    $null = New-Item -ItemType Directory -Path $testRoot
    $script:examples = $testRoot
    $script:releaseDir = Join-Path $testRoot 'release'
    $null = New-Item -ItemType Directory -Path $script:releaseDir
    $controllerElf = Join-Path $script:releaseDir 'i2c-one-shot-rxstall-controller'
    $targetElf = Join-Path $script:releaseDir 'i2c-one-shot-rxstall-target'
    Set-Content -LiteralPath $controllerElf -Value 'controller test elf' -NoNewline
    Set-Content -LiteralPath $targetElf -Value 'target test elf' -NoNewline
    $script:ControllerProbe = 'controller-probe'
    $script:TargetProbe = 'target-probe'
    $script:SpeedKhz = 4000

    # Assert the command contracts independently.  The later Run-Mode tests
    # replace these helpers with fakes so no process or board can be reached.
    $script:boundedCalls = [System.Collections.Generic.List[object]]::new()
    function script:Invoke-BoundedProcess([string]$Program, [string[]]$Arguments, [string]$LogStem, [int]$TimeoutMs) {
        $script:boundedCalls.Add([pscustomobject]@{
            program = $Program; arguments = @($Arguments); log_stem = $LogStem; timeout_ms = $TimeoutMs
        })
    }
    Import-RunnerFunction 'Prepare-Board'
    Prepare-Board 'prepare-probe' $controllerElf (Join-Path $testRoot 'controller-download')
    Check ($script:boundedCalls.Count -eq 1) 'Prepare-Board makes one bounded command'
    $prepareCall = $script:boundedCalls[0]
    Check ($prepareCall.program -eq 'probe-rs' -and $prepareCall.timeout_ms -eq 60000) 'Prepare-Board uses bounded probe-rs download'
    $expectedPrepare = @('download', '--chip', 'MCXA577', '--probe', 'prepare-probe', '--protocol', 'swd', '--speed', '4000',
        '--non-interactive', '--preverify', '--verify', ('"' + $controllerElf + '"'))
    Check (([string]::Join("`0", $prepareCall.arguments)) -eq ([string]::Join("`0", $expectedPrepare))) 'Prepare-Board preloads and verifies the exact ELF'

    $script:startProcessCalls = [System.Collections.Generic.List[object]]::new()
    function script:Start-Process {
        param($FilePath, $ArgumentList, $WorkingDirectory, $WindowStyle, [switch]$PassThru, $RedirectStandardOutput, $RedirectStandardError)
        $script:startProcessCalls.Add([pscustomobject]@{
            file_path = $FilePath; arguments = @($ArgumentList); working_directory = $WorkingDirectory
            stdout = $RedirectStandardOutput; stderr = $RedirectStandardError
        })
        return New-FakeProcess
    }
    Import-RunnerFunction 'Start-Board'
    $null = Start-Board 'start-probe' $targetElf (Join-Path $testRoot 'target')
    Check ($script:startProcessCalls.Count -eq 1) 'Start-Board makes one process launch'
    $startCall = $script:startProcessCalls[0]
    Check ($startCall.file_path -eq 'probe-rs' -and $startCall.working_directory -eq $script:examples) 'Start-Board uses probe-rs in examples directory'
    $expectedStart = @('run', '--chip', 'MCXA577', '--probe', 'start-probe', '--protocol', 'swd', '--speed', '4000',
        '--non-interactive', '--preverify', '--verify', '--scan-region', 'ram', ('"' + $targetElf + '"'))
    Check (([string]::Join("`0", $startCall.arguments)) -eq ([string]::Join("`0", $expectedStart))) 'Start-Board re-verifies the prepared exact ELF'

    Import-RunnerFunction 'Run-Mode'

    $runModeAst = @($script:runnerAst.FindAll({
        param($node)
        $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Run-Mode'
    }, $true))
    $prepareCalls = @($runModeAst[0].FindAll({
        param($node)
        $node -is [System.Management.Automation.Language.CommandAst] -and $node.GetCommandName() -eq 'Prepare-Board'
    }, $true))
    $startupAssignments = @($runModeAst[0].FindAll({
        param($node)
        $node -is [System.Management.Automation.Language.AssignmentStatementAst] -and $node.Left.Extent.Text -eq '$startup'
    }, $true))
    Check ($prepareCalls.Count -eq 2 -and $startupAssignments.Count -eq 1 -and
        (@($prepareCalls | Where-Object { $_.Extent.StartOffset -ge $startupAssignments[0].Extent.StartOffset }).Count -eq 0)) `
        'source keeps both preparations outside the startup stopwatch'

    function script:Reset-Scenario([string]$Name, [string]$MissingReady = $null, [string]$PrepareFailure = $null) {
        $script:logs = Join-Path $testRoot $Name
        $null = New-Item -ItemType Directory -Path $script:logs
        $script:run = [ordered]@{ modes = [System.Collections.Generic.List[object]]::new() }
        $script:boardsTouched = $false
        $script:events = [System.Collections.Generic.List[string]]::new()
        $script:prepareFailure = $PrepareFailure
        $script:boardLogs = @{
            controller = "0.0 [INFO ] RXSTALL CONTROLLER: 10s quiet window`n0.0 [INFO ] RXSTALL SHORT CONTROLLER PASS`n"
            target = if ($MissingReady) { '' } else { "0.0 [INFO ] RXSTALL TARGET READY: short`n0.0 [INFO ] RXSTALL SHORT TARGET PASS: exact 16 bytes and STOP`n" }
        }
    }

    function script:Get-SourceSnapshot { return [ordered]@{ commit = 'host-test' } }
    function script:Build-Bins([string[]]$Bins, [string]$LogStem) { $script:events.Add('build') }
    function script:Prepare-Board([string]$Probe, [string]$Elf, [string]$LogStem) {
        $script:events.Add("prepare:$($Probe):$Elf")
        if ($script:prepareFailure -eq $Probe) { throw "injected preparation failure for $Probe" }
    }
    function script:Reset-Board([string]$Probe, [string]$LogStem) { $script:events.Add("reset:$Probe") }
    function script:Start-Board([string]$Probe, [string]$Elf, [string]$LogStem) {
        $script:events.Add("start:$($Probe):$Elf")
        return New-FakeProcess
    }
    function script:Read-BoardLog([string]$Directory, [string]$Role) { return [string]$script:boardLogs[$Role] }
    function script:Get-RxstallMarkers([string]$Mode) {
        return @{ controller = "RXSTALL $($Mode.ToUpperInvariant()) CONTROLLER PASS"; target = "RXSTALL $($Mode.ToUpperInvariant()) TARGET PASS: exact 16 bytes and STOP" }
    }
    function script:Assert-RxstallEndpoint([string]$Role, [int]$ExitCode, [string]$Log, [string]$Marker) {
        if ($ExitCode -ne 0 -or -not $Log.Contains($Marker)) { throw "bad fake endpoint $Role" }
    }
    function script:Get-RxstallObservations([string]$Mode, [string]$ControllerLog, [string]$TargetLog) { return @{ mode = $Mode } }
    function script:Stop-OwnedProcess($Process) { }

    Reset-Scenario 'success'
    # The fake endpoints must not print a real-board PASS banner in host logs.
    Run-Mode 'short' 6>$null
    $expectedEvents = @(
        'build',
        "prepare:controller-probe:$controllerElf", "prepare:target-probe:$targetElf",
        'reset:controller-probe', 'reset:target-probe',
        "start:controller-probe:$controllerElf", "start:target-probe:$targetElf"
    )
    Check (([string]::Join("`0", $script:events)) -eq ([string]::Join("`0", $expectedEvents))) 'both prepares precede every reset and start'
    Check ($script:run.modes[0].preparation.result -eq 'pass' -and $script:run.modes[0].result -eq 'pass') 'prepared successful mode records success'
    Check ($script:boardsTouched) 'preparation marks boards touched for normal restore'

    foreach ($failedProbe in @('controller-probe', 'target-probe')) {
        Reset-Scenario "prepare-fail-$failedProbe" $null $failedProbe
        RejectWithMessage { Run-Mode 'short' } "injected preparation failure for $failedProbe" "$failedProbe preparation failure rejects the mode"
        Check ($script:run.modes[0].preparation.result -eq 'fail' -and $script:run.modes[0].result -eq 'fail') "$failedProbe preparation failure is recorded"
        Check (@($script:events | Where-Object { $_ -like 'reset:*' -or $_ -like 'start:*' }).Count -eq 0) "$failedProbe preparation failure starts no board"
    }

    Reset-Scenario 'missing-ready' 'missing'
    RejectWithMessage { Run-Mode 'short' } 'Target did not publish its mode-specific READY marker.' 'missing target READY rejects after both prepared starts'
    Check ($script:run.modes[0].preparation.result -eq 'pass' -and $script:run.modes[0].result -eq 'fail') 'missing READY cannot be recorded as a preparation failure or pass'
    Check (@($script:events | Where-Object { $_ -like 'start:*' }).Count -eq 2) 'missing READY test reached both prepared starts'
} finally {
    if (Test-Path -LiteralPath $testRoot) {
        $resolvedRoot = [System.IO.Path]::GetFullPath($testRoot).TrimEnd('\\')
        $resolvedParent = [System.IO.Path]::GetDirectoryName($resolvedRoot)
        $resolvedLeaf = [System.IO.Path]::GetFileName($resolvedRoot)
        if ($resolvedParent -ine $testParent -or $resolvedLeaf -cne $testLeaf -or $resolvedLeaf -notmatch '^rxstall-startup-[0-9a-f]{32}$') {
            throw "Refusing to remove unexpected test directory: $resolvedRoot"
        }
        Remove-Item -LiteralPath $resolvedRoot -Recurse -Force
    }
}

Write-Output "RXSTALL startup runner controls: $passed passed"

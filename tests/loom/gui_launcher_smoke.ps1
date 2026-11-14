# All probes run on inactive desktops. No visible mode, desktop switching, or real input.
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$repo = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
if (-not ('Loom.GuiTestProcess' -as [type])) { Add-Type -Path (Join-Path $repo 'scripts/GuiTestProcess.cs') }
$run = Join-Path ([IO.Path]::GetTempPath()) ('loom-launcher-test-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Force $run | Out-Null
$exe = Join-Path $env:WINDIR 'System32/WindowsPowerShell/v1.0/powershell.exe'
$pidPath = (Join-Path $run 'pids.json').Replace("'", "''")
$childExe = $exe.Replace("'", "''")
$script = @'
[Console]::Out.WriteLine('LOOM_STDOUT_OK')
[Console]::Error.WriteLine('LOOM_STDERR_OK')
[Console]::Out.Flush()
[Console]::Error.Flush()
$info = New-Object System.Diagnostics.ProcessStartInfo
$info.FileName = '__EXE__'
$info.Arguments = '-NoLogo -NoProfile -NonInteractive -Command "Start-Sleep -Seconds 120"'
$info.UseShellExecute = $false
$info.CreateNoWindow = $true
$child = [Diagnostics.Process]::Start($info)
[IO.File]::WriteAllText('__PIDS__', (@{ parent = $PID; child = $child.Id } | ConvertTo-Json))
Start-Sleep -Seconds 120
'@
$script = $script.Replace('__EXE__', $childExe).Replace('__PIDS__', $pidPath)
$encoded = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($script))
$process = $null
$parent = $null
$child = $null
try {
    $process = [Loom.GuiTestProcess]::Start($exe, "-NoLogo -NoProfile -NonInteractive -EncodedCommand $encoded", $run,
        ('loom-probe-' + [guid]::NewGuid().ToString('N')), "$run/stdout.log", "$run/stderr.log")
    $deadline = [DateTime]::UtcNow.AddSeconds(15)
    while (-not (Test-Path "$run/pids.json")) {
        if ([DateTime]::UtcNow -ge $deadline) { throw "Probe did not start. Logs: $run" }
        Start-Sleep -Milliseconds 100
    }
    $pids = [IO.File]::ReadAllText("$run/pids.json") | ConvertFrom-Json
    $parent = [Diagnostics.Process]::GetProcessById($pids.parent)
    $child = [Diagnostics.Process]::GetProcessById($pids.child)
    # Hold handles to these exact processes, even if Windows later reuses a PID.
    $null = $parent.Handle
    $null = $child.Handle
    if ($process.WaitForExit(100)) { throw 'Probe exited before timeout test' }
    $process.Dispose()
    $process = $null
    if (-not $parent.WaitForExit(5000)) { throw 'Job cleanup left the parent running' }
    if (-not $child.WaitForExit(5000)) { throw 'Job cleanup left the child running' }
    if (-not [IO.File]::ReadAllText("$run/stdout.log").Contains('LOOM_STDOUT_OK')) { throw 'Missing redirected stdout' }
    if (-not [IO.File]::ReadAllText("$run/stderr.log").Contains('LOOM_STDERR_OK')) { throw 'Missing redirected stderr' }
    Write-Host 'PASS: hidden launch, timeout detection, parent/child job cleanup, and redirected logs.'

    $failed = $false
    try {
        $process = [Loom.GuiTestProcess]::Start("$run/missing.exe", '', $run,
            ('loom-failure-' + [guid]::NewGuid().ToString('N')), "$run/failure-out.log", "$run/failure-err.log")
    }
    catch { $failed = $true }
    if (-not $failed) { throw 'Missing executable unexpectedly launched' }
    Write-Host 'PASS: launch failure is reported without a visible fallback.'
    Write-Host "Disposable launcher logs: $run"
}
finally {
    if ($process) { $process.Dispose() }
    if ($parent) { $parent.Dispose() }
    if ($child) { $child.Dispose() }
}

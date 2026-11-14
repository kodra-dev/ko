# Launches its OWN disposable Houdini on an inactive Win32 desktop by default.
# Never switches desktops. Visible mode is explicit; there is NO visible fallback.
[CmdletBinding()]
param(
    [string]$Houdini = '',
    [ValidateRange(1, 3600)][int]$TimeoutSeconds = 120,
    [switch]$Visible,
    [switch]$PaidProvider,
    [switch]$ConfirmPaidUsage
)
$ErrorActionPreference = 'Stop'
if ($PaidProvider -and -not $ConfirmPaidUsage) { throw 'PAID provider tests consume subscription usage. Obtain approval, then pass -ConfirmPaidUsage.' }
if ($env:OS -ne 'Windows_NT') { throw 'This graphical test launcher requires Windows.' }
$repo = Split-Path $PSScriptRoot -Parent
if (-not $Houdini) {
    $install = Get-ChildItem 'C:/Program Files/Side Effects Software' -Directory |
        Where-Object { $_.Name -match '^Houdini \d+\.\d+\.\d+$' } |
        Sort-Object { [version]($_.Name -replace '^Houdini ', '') } -Descending |
        Where-Object { Test-Path (Join-Path $_.FullName 'bin/houdini.exe') } |
        Select-Object -First 1
    if ($install) { $Houdini = Join-Path $install.FullName 'bin/houdini.exe' }
}
if (-not $Houdini -or -not (Test-Path $Houdini)) { throw 'Pass -Houdini <path to the latest installed houdini.exe>' }
$Houdini = (Resolve-Path $Houdini).Path
if (-not ('Loom.GuiTestProcess' -as [type])) { Add-Type -Path (Join-Path $PSScriptRoot 'GuiTestProcess.cs') }
$run = Join-Path ([IO.Path]::GetTempPath()) ('loom-gui-test-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Force (Join-Path $run 'package/scripts') | Out-Null
# Install only Loom's menu overlay into the disposable package, not all ko assets.
Copy-Item (Join-Path $repo 'MainMenuCommon.xml') (Join-Path $run 'package/MainMenuCommon.xml')
$startupFile = if ($PaidProvider) { 'tests/loom/paid/gui_startup_paid.py' } else { 'tests/loom/gui_startup.py' }
$startup = (Join-Path $repo $startupFile).Replace('\', '/')
Set-Content -Path (Join-Path $run 'package/scripts/123.py') -Encoding utf8 -Value "import runpy; runpy.run_path(r'''$startup''')"
$desktopName = if ($Visible) { $null } else { 'loom-test-' + [guid]::NewGuid().ToString('N') }
$mode = if ($Visible) { 'visible (explicit opt-in)' } else { 'hidden desktop (no switching)' }
$names = @('HOUDINI_USER_PREF_DIR','HOUDINI_PATH','HOUDINI_MENU_PATH','HOUDINI_NO_ENV_FILE','KO_AI_HOME','LOOM_HOME','LOOM_GUI_REPO','LOOM_GUI_TEST_DIR','LOOM_GUI_TEST_DESKTOP','LOOM_GUI_TEST_AUTH_ROOT','LOOM_BLOCK_PROVIDER','LOOM_ALLOW_PAID_TESTS','QT_QPA_PLATFORM')
$old = @{}
foreach ($name in $names) { $old[$name] = [Environment]::GetEnvironmentVariable($name, 'Process') }
$process = $null
$launch = [ordered]@{ mode = $mode; desktop = $desktopName; process_id = $null; input_before = $null; input_after = $null; exit_code = $null }
try { $launch.input_before = [Loom.GuiTestProcess]::InputDesktopName() } catch { $launch.input_before = $_.Exception.Message }
try {
    try {
        # __HVER__ is required; otherwise Houdini may ignore this preference override.
        $env:HOUDINI_USER_PREF_DIR = "$run/prefs/houdini__HVER__"
        $env:HOUDINI_PATH = "$run/package;&"
        $env:HOUDINI_MENU_PATH = "$run/package;&"
        $env:HOUDINI_NO_ENV_FILE = '1'
        # PAID scenarios live separately; the normal renderer cannot start a provider.
        $env:LOOM_BLOCK_PROVIDER = if ($PaidProvider) { "0" } else { "1" }
        $env:LOOM_ALLOW_PAID_TESTS = if ($PaidProvider) { "1" } else { $null }
        $env:LOOM_GUI_TEST_AUTH_ROOT = if ($PaidProvider) {
            if ($old['LOOM_HOME']) { $old['LOOM_HOME'] }
            elseif ($old['KO_AI_HOME']) { $old['KO_AI_HOME'] }
            elseif (Test-Path (Join-Path $env:LOCALAPPDATA 'Loom')) { Join-Path $env:LOCALAPPDATA 'Loom' }
            else { Join-Path $env:LOCALAPPDATA 'ko-ai' }
        } else { $null }
        Remove-Item Env:KO_AI_HOME -ErrorAction SilentlyContinue
        $env:LOOM_HOME = "$run/data"
        $env:LOOM_GUI_REPO = $repo
        $env:LOOM_GUI_TEST_DIR = $run
        $env:LOOM_GUI_TEST_DESKTOP = $desktopName
        Remove-Item Env:QT_QPA_PLATFORM -ErrorAction SilentlyContinue
        Write-Host "Testing graphical Houdini: $Houdini"
        Write-Host "Mode: $mode"
        Write-Host "Disposable files and logs: $run"
        $process = [Loom.GuiTestProcess]::Start($Houdini, '-foreground -n', $run, $desktopName, "$run/stdout.log", "$run/stderr.log")
        $launch.process_id = $process.ProcessId
    }
    finally {
        foreach ($name in $names) { [Environment]::SetEnvironmentVariable($name, $old[$name], 'Process') }
    }
    if (-not $process.WaitForExit($TimeoutSeconds * 1000)) {
        throw "Disposable GUI test timed out ($mode). No visible fallback. Logs: $run"
    }
    $launch.exit_code = $process.ExitCode
    $resultPath = Join-Path $run 'result.json'
    if (-not (Test-Path $resultPath)) { throw "GUI test ended without a result ($mode). No visible fallback. Logs: $run" }
    $result = [IO.File]::ReadAllText($resultPath) | ConvertFrom-Json
    if (-not $result.passed) { throw "GUI test failed ($mode): $($result.error)`nNo visible fallback. Logs: $run" }
    if ($desktopName -and $result.desktop -ne $desktopName) { throw "GUI ran on the wrong desktop. Expected $desktopName, got $($result.desktop). Logs: $run" }
    if ($process.ExitCode -ne 0) { throw "GUI test exited with code $($process.ExitCode). Logs: $run" }
    Write-Host "PASS ($mode): native panes, conversation manager/F2/activation/delete, reloads, real viewport capture, Save/Save As sync, and shared controller lifecycle ($($result.houdini))."
}
finally {
    # Kill only this disposable job's remaining children, then release all handles.
    if ($process) { $process.Dispose() }
    try { $launch.input_after = [Loom.GuiTestProcess]::InputDesktopName() } catch { $launch.input_after = $_.Exception.Message }
    $launch | ConvertTo-Json | Set-Content -Path (Join-Path $run 'launcher.json') -Encoding utf8
}

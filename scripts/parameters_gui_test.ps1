# Offline native Toggle Parameters regression on an inactive Win32 desktop.
# No visible fallback, global input, provider, or artist preferences.
[CmdletBinding()]
param(
    [string]$Houdini = 'C:/Program Files/Side Effects Software/Houdini 22.0.429/bin/houdini.exe',
    [ValidateRange(1, 3600)][int]$TimeoutSeconds = 120
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
if (-not (Test-Path $Houdini)) { throw 'Pass -Houdini <path to houdini.exe>' }
if (-not ('Loom.GuiTestProcess' -as [type])) { Add-Type -Path "$PSScriptRoot/GuiTestProcess.cs" }
$run = Join-Path ([IO.Path]::GetTempPath()) ('ko-parameters-gui-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Force "$run/package/scripts" | Out-Null
Copy-Item (Join-Path $repo 'MainMenuCommon.xml') "$run/package/MainMenuCommon.xml"
$startup = (Join-Path $repo 'tests/ko/parameters_gui_startup.py').Replace('\', '/')
Set-Content "$run/package/scripts/123.py" -Encoding utf8 "import runpy;runpy.run_path(r'''$startup''')"
$names = @('HOUDINI_USER_PREF_DIR','HOUDINI_PATH','HOUDINI_MENU_PATH','HOUDINI_NO_ENV_FILE',
           'KO_PARAMETERS_GUI_RUN','KO_PARAMETERS_GUI_REPO','KO_PARAMETERS_GUI_DESKTOP',
           'KO_PARAMETERS_STATE','LOOM_BLOCK_PROVIDER','QT_QPA_PLATFORM')
$old = @{}
foreach ($name in $names) { $old[$name] = [Environment]::GetEnvironmentVariable($name, 'Process') }
$process = $null
$before = [Loom.GuiTestProcess]::InputDesktopName()
$desktop = 'ko-parameters-test-' + [guid]::NewGuid().ToString('N')
try {
    try {
        $env:HOUDINI_USER_PREF_DIR = "$run/prefs/houdini__HVER__"
        $env:HOUDINI_PATH = "$run/package;&"
        $env:HOUDINI_MENU_PATH = "$run/package;&"
        $env:HOUDINI_NO_ENV_FILE = '1'
        $env:KO_PARAMETERS_GUI_RUN = $run
        $env:KO_PARAMETERS_GUI_REPO = $repo
        $env:KO_PARAMETERS_GUI_DESKTOP = $desktop
        $env:KO_PARAMETERS_STATE = "$run/parameters.json"
        $env:LOOM_BLOCK_PROVIDER = '1'
        Remove-Item Env:QT_QPA_PLATFORM -ErrorAction SilentlyContinue
        Write-Host "Hidden Toggle Parameters test logs: $run"
        $process = [Loom.GuiTestProcess]::Start($Houdini, '-foreground -n', $run, $desktop,
                                               "$run/stdout.log", "$run/stderr.log")
    }
    finally {
        foreach ($name in $names) { [Environment]::SetEnvironmentVariable($name, $old[$name], 'Process') }
    }
    if (-not $process.WaitForExit($TimeoutSeconds * 1000)) { throw "Hidden test timed out. Logs: $run" }
    if (-not (Test-Path "$run/result.json")) { throw "Missing GUI result. Logs: $run" }
    $result = [IO.File]::ReadAllText("$run/result.json") | ConvertFrom-Json
    if (-not $result.passed -or $process.ExitCode -ne 0) { throw "GUI test failed: $($result.error). Logs: $run" }
    if ($result.desktop -ne $desktop) { throw "Wrong test desktop. Logs: $run" }
    if ([Loom.GuiTestProcess]::InputDesktopName() -ne $before) { throw 'Input desktop unexpectedly changed' }
    Write-Host "PASS hidden Toggle Parameters ($($result.houdini)): $($result.checks -join ', ')"
}
finally {
    if ($process) { $process.Dispose() }
}

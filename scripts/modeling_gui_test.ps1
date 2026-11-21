# Offline modeling smoke on an inactive Win32 desktop, with isolated preferences.
[CmdletBinding()]
param(
    [string]$Houdini = 'C:/Program Files/Side Effects Software/Houdini 22.0.466/bin/houdini.exe',
    [ValidateRange(1, 3600)][int]$TimeoutSeconds = 90
)
$ErrorActionPreference = 'Stop'
if (-not (Test-Path $Houdini)) { throw 'Pass -Houdini <path to houdini.exe>' }
if (-not ('Loom.GuiTestProcess' -as [type])) { Add-Type -Path "$PSScriptRoot/GuiTestProcess.cs" }
$repo = Split-Path $PSScriptRoot -Parent
$run = Join-Path ([IO.Path]::GetTempPath()) ('ko-modeling-gui-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Force "$run/package/scripts", "$run/package/python3.13libs", "$run/package/radialmenu", "$run/package/otls", "$run/package/viewer_states", "$run/package/python_panels" | Out-Null
Copy-Item "$repo/python_panels/ko_adjust_parts.pypanel" "$run/package/python_panels"
Copy-Item "$repo/viewer_states/ko_adjust_viewport.py" "$run/package/viewer_states"
Copy-Item "$repo/python3.13libs/ko_adjust*.py" "$run/package/python3.13libs"
Copy-Item "$repo/radialmenu/ko_adjust.radialmenu" "$run/package/radialmenu"
Copy-Item "$repo/otls/sop_ko.koadjust.1.0.hda" "$run/package/otls"
$startup = (Join-Path $repo 'tests/ko/modeling_gui_startup.py').Replace('\', '/')
Set-Content "$run/package/scripts/123.py" -Encoding utf8 "import runpy;runpy.run_path(r'''$startup''')"
$envs = @{
    HOUDINI_USER_PREF_DIR = "$run/prefs/houdini__HVER__"
    HOUDINI_PATH = "$run/package;&"
    HOUDINI_NO_ENV_FILE = '1'
    KO_MODEL_TEST_RUN = $run
    KO_MODEL_TEST_REPO = $repo
    LOOM_BLOCK_PROVIDER = '1'
    QT_QPA_PLATFORM = $null
}
$desktop = 'ko-modeling-test-' + [guid]::NewGuid().ToString('N')
$before = [Loom.GuiTestProcess]::InputDesktopName()
$old = @{}
$process = $null
try {
    try {
        foreach ($key in $envs.Keys) {
            $old[$key] = [Environment]::GetEnvironmentVariable($key, 'Process')
            [Environment]::SetEnvironmentVariable($key, $envs[$key], 'Process')
        }
        Write-Host "Hidden modeling test logs: $run"
        $process = [Loom.GuiTestProcess]::Start($Houdini, '-foreground -n', $run, $desktop,
                                               "$run/stdout.log", "$run/stderr.log")
    } finally {
        foreach ($key in $old.Keys) { [Environment]::SetEnvironmentVariable($key, $old[$key], 'Process') }
    }
    if (-not $process.WaitForExit($TimeoutSeconds * 1000)) { throw "Test timed out. Logs: $run" }
    if (-not (Test-Path "$run/result.json")) { throw "Missing GUI result. Logs: $run" }
    $result = [IO.File]::ReadAllText("$run/result.json") | ConvertFrom-Json
    if ($process.ExitCode -ne 0 -or $result.error) { throw "GUI test failed: $($result.error). Logs: $run" }
    if ($result.desktop -ne $desktop) { throw 'Wrong test desktop' }
    if ([Loom.GuiTestProcess]::InputDesktopName() -ne $before) { throw 'Input desktop unexpectedly changed' }
    Write-Host "PASS hidden modeling GUI ($($result.houdini)): $($result.checks -join ', ')"
} finally {
    if ($process) { $process.Dispose() }
}

# Launches its OWN disposable GUI session; never connects to or closes your Houdini.
[CmdletBinding()]
param([string]$Houdini = '', [int]$TimeoutSeconds = 120)
$ErrorActionPreference = 'Stop'
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
$run = Join-Path ([IO.Path]::GetTempPath()) ('ko-ai-gui-test-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Force (Join-Path $run 'package/scripts') | Out-Null
$startup = (Join-Path $repo 'tests/ko_ai/gui_startup.py').Replace('\', '/')
Set-Content -Path (Join-Path $run 'package/scripts/123.py') -Encoding utf8 -Value "import runpy; runpy.run_path(r'''$startup''')"
$names = @('HOUDINI_USER_PREF_DIR','HOUDINI_PATH','HOUDINI_NO_ENV_FILE','KO_AI_HOME','KO_AI_GUI_REPO','KO_AI_GUI_TEST_DIR','QT_QPA_PLATFORM')
$old = @{}
foreach ($name in $names) { $old[$name] = [Environment]::GetEnvironmentVariable($name, 'Process') }
try {
    # __HVER__ is required; otherwise Houdini may ignore this preference override.
    $env:HOUDINI_USER_PREF_DIR = "$run/prefs/houdini__HVER__"
    $env:HOUDINI_PATH = "$run/package;&"
    $env:HOUDINI_NO_ENV_FILE = '1'
    $env:KO_AI_HOME = "$run/data"
    $env:KO_AI_GUI_REPO = $repo
    $env:KO_AI_GUI_TEST_DIR = $run
    Remove-Item Env:QT_QPA_PLATFORM -ErrorAction SilentlyContinue
    Write-Host "Testing graphical Houdini: $Houdini"
    Write-Host "Disposable files and logs: $run"
    $process = Start-Process $Houdini -ArgumentList '-foreground','-n' -WorkingDirectory $run -PassThru `
        -RedirectStandardOutput "$run/stdout.log" -RedirectStandardError "$run/stderr.log"
}
finally {
    foreach ($name in $names) { [Environment]::SetEnvironmentVariable($name, $old[$name], 'Process') }
}
if (-not $process.WaitForExit($TimeoutSeconds * 1000)) {
    Stop-Process -Id $process.Id -ErrorAction SilentlyContinue
    throw "Disposable GUI test timed out. Logs: $run"
}
$resultPath = Join-Path $run 'result.json'
if (-not (Test-Path $resultPath)) { throw "GUI test ended without a result. Logs: $run" }
$result = [IO.File]::ReadAllText($resultPath) | ConvertFrom-Json
if (-not $result.passed) { throw "GUI test failed: $($result.error)`nLogs: $run" }
Write-Host "PASS: graphical pane creation, reload button, direct/native reloads, persisted history/chat, duplicate notice, and broken-binding recovery ($($result.houdini))."

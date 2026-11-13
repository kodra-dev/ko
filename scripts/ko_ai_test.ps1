[CmdletBinding()]
param(
    [string]$Hython = '',
    [switch]$Chat
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
if (-not $Hython) {
    $install = Get-ChildItem 'C:/Program Files/Side Effects Software' -Directory -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -match '^Houdini \d+\.\d+\.\d+$' } |
        Sort-Object { [version]($_.Name -replace '^Houdini ', '') } -Descending |
        Where-Object { Test-Path (Join-Path $_.FullName 'bin/hython.exe') } |
        Select-Object -First 1
    if ($install) { $Hython = Join-Path $install.FullName 'bin/hython.exe' }
    else {
        $found = Get-Command hython -ErrorAction SilentlyContinue
        if ($found) { $Hython = $found.Source }
    }
}
if (-not $Hython -or -not (Test-Path $Hython)) { throw 'Pass -Hython <path to the latest installed hython.exe>' }
Write-Host "Testing Houdini: $Hython"
$oldPythonPath = $env:PYTHONPATH
$oldQtPlatform = $env:QT_QPA_PLATFORM
try {
    $env:PYTHONPATH = Join-Path $repo 'python3.13libs'
    & $Hython -m unittest discover -s (Join-Path $repo 'tests/ko_ai') -p 'test_*.py' -v
    if ($LASTEXITCODE -ne 0) { throw 'Unit tests failed' }
    & $Hython (Join-Path $repo 'tests/ko_ai/houdini_smoke.py')
    if ($LASTEXITCODE -ne 0) { throw 'Houdini integration tests failed' }
    $env:QT_QPA_PLATFORM = 'offscreen'
    & $Hython (Join-Path $repo 'tests/ko_ai/ui_smoke.py')
    if ($LASTEXITCODE -ne 0) { throw 'Qt UI tests failed' }
    $arguments = @((Join-Path $repo 'tests/ko_ai/provider_smoke.py'))
    if ($Chat) { $arguments += '--chat' }
    & $Hython @arguments
    if ($LASTEXITCODE -ne 0) { throw 'Codex protocol smoke test failed' }
    Write-Host 'All tests passed. Tests used disposable scenes, not your open Houdini session.'
}
finally {
    $env:PYTHONPATH = $oldPythonPath
    $env:QT_QPA_PLATFORM = $oldQtPlatform
}

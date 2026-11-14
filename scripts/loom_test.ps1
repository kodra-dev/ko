[CmdletBinding()]
param(
    [string]$Hython = ''
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
$oldLoomHome = $env:LOOM_HOME
$oldBlockProvider = $env:LOOM_BLOCK_PROVIDER
try {
    # OFFLINE ONLY. Tests supply isolated LOOM_HOME directories.
    $env:LOOM_BLOCK_PROVIDER = "1"
    Remove-Item Env:LOOM_HOME -ErrorAction SilentlyContinue
    $env:PYTHONPATH = Join-Path $repo 'python3.13libs'
    & $Hython -m unittest discover -s (Join-Path $repo 'tests/loom') -p 'test_*.py' -v
    if ($LASTEXITCODE -ne 0) { throw 'Unit tests failed' }
    & $Hython (Join-Path $repo 'tests/loom/houdini_smoke.py')
    if ($LASTEXITCODE -ne 0) { throw 'Houdini integration tests failed' }
    $env:QT_QPA_PLATFORM = 'offscreen'
    & $Hython (Join-Path $repo 'tests/loom/ui_smoke.py')
    if ($LASTEXITCODE -ne 0) { throw 'Qt UI tests failed' }
    & $Hython (Join-Path $repo 'tests/loom/ui_package_reload.py')
    if ($LASTEXITCODE -ne 0) { throw 'Package reload tests failed' }
    Write-Host 'All tests passed. Tests used disposable scenes, not your open Houdini session.'
}
finally {
    $env:PYTHONPATH = $oldPythonPath
    $env:QT_QPA_PLATFORM = $oldQtPlatform
    $env:LOOM_HOME = $oldLoomHome
    $env:LOOM_BLOCK_PROVIDER = $oldBlockProvider
}

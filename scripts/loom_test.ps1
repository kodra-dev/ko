[CmdletBinding()]
param(
    [string]$Hython = '',
    [ValidateSet('Full', 'Fast', 'Unit', 'Houdini', 'UI', 'Reload')]
    [string]$Suite = 'Full',
    [string[]]$Test = @(),
    [string]$Python = 'python'
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
if ($Test.Count -and $Suite -eq 'Full') { throw '-Test requires a specific -Suite (Fast, Unit, Houdini, UI, or Reload).' }
if ($Suite -ne 'Fast' -and -not $Hython) {
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
if ($Suite -eq 'Fast') {
    $found = Get-Command $Python -ErrorAction SilentlyContinue
    if (-not $found) { throw 'Pass -Python <path to an ordinary Python 3 interpreter> for the Fast suite.' }
    $executable = $found.Source
} else {
    if (-not $Hython -or -not (Test-Path $Hython)) { throw 'Pass -Hython <path to the latest installed hython.exe>' }
    $executable = $Hython
}
Write-Host "Testing with: $executable"
$suites = if ($Suite -eq 'Full') { @('unit', 'houdini', 'ui', 'reload') } else { @($Suite.ToLowerInvariant()) }
$total = [Diagnostics.Stopwatch]::StartNew()
$oldPythonPath = $env:PYTHONPATH
$oldQtPlatform = $env:QT_QPA_PLATFORM
$oldLoomHome = $env:LOOM_HOME
$oldBlockProvider = $env:LOOM_BLOCK_PROVIDER
try {
    # OFFLINE ONLY. The runner supplies a disposable LOOM_HOME before imports.
    $env:LOOM_BLOCK_PROVIDER = "1"
    Remove-Item Env:LOOM_HOME -ErrorAction SilentlyContinue
    $env:PYTHONPATH = Join-Path $repo 'python3.13libs'
    $env:QT_QPA_PLATFORM = 'offscreen'
    foreach ($selected in $suites) {
        $arguments = @((Join-Path $repo 'scripts/loom_test_runner.py'), '--suite', $selected)
        foreach ($selector in $Test) { $arguments += @('--test', $selector) }
        $timer = [Diagnostics.Stopwatch]::StartNew()
        try {
            & $executable @arguments
            if ($LASTEXITCODE -ne 0) { throw "$selected tests failed (exit $LASTEXITCODE)" }
        }
        finally {
            Write-Host ('{0}: wall {1:N3}s (includes interpreter startup and cleanup)' -f $selected, $timer.Elapsed.TotalSeconds)
        }
    }
    Write-Host 'All tests passed. Tests used disposable scenes, not your open Houdini session.'
}
finally {
    Write-Host ('Total wall: {0:N3}s' -f $total.Elapsed.TotalSeconds)
    $env:PYTHONPATH = $oldPythonPath
    $env:QT_QPA_PLATFORM = $oldQtPlatform
    $env:LOOM_HOME = $oldLoomHome
    $env:LOOM_BLOCK_PROVIDER = $oldBlockProvider
}

# PAID: real ChatGPT subscription usage. Never invoked by default tests.
[CmdletBinding()]
param([switch]$ConfirmPaidUsage, [switch]$Scene, [string]$Hython = '')
$ErrorActionPreference = 'Stop'
if (-not $ConfirmPaidUsage) { throw 'Obtain explicit user approval, then pass -ConfirmPaidUsage. This consumes subscription usage.' }
if (-not $Hython) {
    $install = Get-ChildItem 'C:/Program Files/Side Effects Software' -Directory |
        Where-Object { $_.Name -match '^Houdini \d+\.\d+\.\d+$' } |
        Sort-Object { [version]($_.Name -replace '^Houdini ', '') } -Descending |
        Select-Object -First 1
    $Hython = Join-Path $install.FullName 'bin/hython.exe'
}
$oldBlock = $env:LOOM_BLOCK_PROVIDER
$oldQt = $env:QT_QPA_PLATFORM
try {
    $env:LOOM_BLOCK_PROVIDER = '0'
    $env:QT_QPA_PLATFORM = 'offscreen'
    $test = if ($Scene) { 'scene_roundtrip.py' } else { 'provider_roundtrip.py' }
    Write-Host "PAID provider test: $test"
    & $Hython (Join-Path $PSScriptRoot $test) --allow-paid-provider
    if ($LASTEXITCODE -ne 0) { throw 'Paid provider test failed' }
} finally {
    $env:LOOM_BLOCK_PROVIDER = $oldBlock
    $env:QT_QPA_PLATFORM = $oldQt
}

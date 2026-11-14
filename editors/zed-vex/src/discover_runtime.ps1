# Host-side discovery only. Stdout is one JSON object, never an LSP proxy.
# Uses Houdini's regular Python, never hython/hou or a license-consuming process.
param([string]$HoudiniPath = $env:KO_VEX_EXPLICIT_HFS, [string[]]$SearchRoot = @())
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
if (-not $SearchRoot.Count) {
    $SearchRoot = @($env:ProgramW6432, $env:ProgramFiles) | Where-Object { $_ } | Select-Object -Unique
    $SearchRoot = @($SearchRoot | ForEach-Object { Join-Path $_ 'Side Effects Software' })
}
$explicit = -not [string]::IsNullOrWhiteSpace($HoudiniPath)
if ($explicit) {
    $candidates = @([Environment]::ExpandEnvironmentVariables($HoudiniPath))
} elseif (-not $PSBoundParameters.ContainsKey('SearchRoot') -and $env:HFS -and (Test-Path -LiteralPath (Join-Path $env:HFS 'bin/vcc.exe'))) {
    $candidates = @($env:HFS)
} else {
    $candidates = @($SearchRoot | Where-Object { Test-Path -LiteralPath $_ } | ForEach-Object {
        Get-ChildItem -LiteralPath $_ -Directory -Filter 'Houdini *' | Where-Object {
            $_.Name -match '^Houdini \d+\.\d+(?:\.\d+)?$' -and (Test-Path -LiteralPath (Join-Path $_.FullName 'bin/vcc.exe'))
        }
    } | Sort-Object { [version]($_.Name.Substring(8)) } -Descending | ForEach-Object { $_.FullName })
}
foreach ($hfs in $candidates) {
    if (-not (Test-Path -LiteralPath (Join-Path $hfs 'bin/vcc.exe'))) { continue }
    $pythons = @(Get-ChildItem -LiteralPath $hfs -Directory -Filter 'python*' | Where-Object {
        $_.Name -match '^python3(\d+)$' -and [int]$Matches[1] -ge 10
    } | Sort-Object { [int]($_.Name.Substring(6)) } -Descending | ForEach-Object {
        Join-Path $_.FullName 'python.exe'
    })
    foreach ($python in $pythons) {
        if (Test-Path -LiteralPath $python -PathType Leaf) {
            @{ python=$python; houdiniPath=$hfs } | ConvertTo-Json -Compress
            exit 0
        }
    }
}
if ($explicit) { throw "No supported bundled Python found in Houdini installation '$HoudiniPath'. Set lsp.ko-vex.settings.pythonPath if needed." }
throw 'No Houdini installation with Python 3.10+ found. Set lsp.ko-vex.settings.houdiniPath (or pythonPath for a custom interpreter).'

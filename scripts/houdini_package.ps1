# DEVELOPER packaging only. Users install the prepared extension once in Zed;
# no project setup, Python install, server command, or scene connection is needed.
[CmdletBinding()]
param([string]$Destination)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$defaultDestination = Join-Path $repo 'build/zed-houdini'
if ([string]::IsNullOrWhiteSpace($Destination)) { $Destination = $defaultDestination }
$Destination = [IO.Path]::GetFullPath($Destination).TrimEnd([IO.Path]::DirectorySeparatorChar, [IO.Path]::AltDirectorySeparatorChar)
$insideRepo = $Destination.Equals($repo, [StringComparison]::OrdinalIgnoreCase) -or
    $Destination.StartsWith($repo + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)
if ($insideRepo -and -not $Destination.Equals($defaultDestination, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'In-repository output must use the gitignored build/zed-houdini directory, not a source directory.'
}
New-Item -ItemType Directory -Force $Destination | Out-Null
$grammar = Join-Path $Destination 'grammar'
New-Item -ItemType Directory -Force $grammar | Out-Null
# Regeneration must remove deleted sources too, while retaining Git/build caches.
if (Test-Path (Join-Path $grammar 'src')) { Remove-Item (Join-Path $grammar 'src') -Recurse -Force }
foreach ($name in @('src', 'grammar.js', 'TREE_SITTER_LICENSE')) {
    Copy-Item (Join-Path $repo "editors/tree-sitter-vex/$name") $grammar -Recurse -Force
}
# Zed's dev builder fetches grammar sources through Git even for local URLs.
# This is an isolated artifact cache; no commits/pushes in the user's repository.
if (-not (Test-Path (Join-Path $grammar '.git'))) {
    & git -C $grammar init --quiet
    if ($LASTEXITCODE) { throw 'Grammar cache init failed' }
}
& git -C $grammar -c core.autocrlf=false add src grammar.js TREE_SITTER_LICENSE
if ($LASTEXITCODE) { throw 'Grammar cache staging failed' }
& git -C $grammar diff --cached --quiet
if ($LASTEXITCODE -eq 1) {
    & git -C $grammar -c user.name='Ko VEX cache' -c user.email='vex-cache@localhost' -c commit.gpgsign=false commit --quiet -m 'Update local VEX grammar'
    if ($LASTEXITCODE) { throw 'Grammar cache commit failed' }
} elseif ($LASTEXITCODE -ne 0) { throw 'Grammar cache diff failed' }
$revision = (& git -C $grammar rev-parse HEAD).Trim()
if ($LASTEXITCODE) { throw 'Grammar cache revision lookup failed' }
$extension = Join-Path $Destination 'zed-extension'
New-Item -ItemType Directory -Force $extension | Out-Null
foreach ($name in @('src', 'languages', 'server')) {
    $generatedSource = Join-Path $extension $name
    if (Test-Path $generatedSource) { Remove-Item $generatedSource -Recurse -Force }
}
foreach ($name in @('Cargo.toml','Cargo.lock','rust-toolchain.toml','build.rs','src','languages')) {
    Copy-Item (Join-Path $repo "editors/zed-houdini/$name") $extension -Recurse -Force
}
foreach ($package in @(@('tools/vex-lsp', 'ko_vex_lsp'), @('tools/apex-lsp', 'ko_apex_lsp'))) {
    $server = Join-Path $extension ('server/' + $package[1])
    New-Item -ItemType Directory -Force $server | Out-Null
    Get-ChildItem (Join-Path $repo ($package[0] + '/' + $package[1])) -File -Filter '*.py' |
        Copy-Item -Destination $server -Force
}
$url = ([uri]($grammar.Replace('\','/') + '/')).AbsoluteUri.TrimEnd('/')
$template = [IO.File]::ReadAllText((Join-Path $repo 'editors/zed-houdini/extension.toml.in'))
# JSON string quoting is also valid TOML basic-string quoting. Preserve the
# script exactly (including line endings), so capability checks match include_str!.
$discovery = [IO.File]::ReadAllText((Join-Path $extension 'src/discover_runtime.ps1')) | ConvertTo-Json -Compress
$manifest = $template.Replace('@GRAMMAR_REPOSITORY@', $url).Replace('@GRAMMAR_REV@', $revision).Replace('@DISCOVERY_SCRIPT@', $discovery)
[IO.File]::WriteAllText((Join-Path $extension 'extension.toml'), $manifest)
Write-Host "Prepared extension: $extension"
Write-Host 'In Zed: Extensions -> Install Dev Extension -> select that folder.'
Write-Host 'Then open VEX (.vfl) or APEX Script (.apex) in any workspace. No per-workspace setup is needed.'

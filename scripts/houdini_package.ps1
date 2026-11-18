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
# Each grammar gets an isolated Git cache: Zed's dev builder fetches grammar sources
# through Git even for local URLs. No commits/pushes in the user's repository.
function Update-GrammarCache([string]$Source, [string]$Cache, [string]$Label) {
    New-Item -ItemType Directory -Force $Cache | Out-Null
    # Regeneration must remove deleted sources too, while retaining Git/build caches.
    if (Test-Path (Join-Path $Cache 'src')) { Remove-Item (Join-Path $Cache 'src') -Recurse -Force }
    foreach ($name in @('src', 'grammar.js', 'TREE_SITTER_LICENSE')) {
        Copy-Item (Join-Path $repo "editors/$Source/$name") $Cache -Recurse -Force
    }
    if (-not (Test-Path (Join-Path $Cache '.git'))) {
        & git -C $Cache init --quiet
        if ($LASTEXITCODE) { throw "$Label grammar cache init failed" }
    }
    & git -C $Cache -c core.autocrlf=false add src grammar.js TREE_SITTER_LICENSE
    if ($LASTEXITCODE) { throw "$Label grammar cache staging failed" }
    & git -C $Cache diff --cached --quiet
    if ($LASTEXITCODE -eq 1) {
        & git -C $Cache -c user.name='Ko grammar cache' -c user.email='grammar-cache@localhost' -c commit.gpgsign=false commit --quiet -m "Update local $Label grammar"
        if ($LASTEXITCODE) { throw "$Label grammar cache commit failed" }
    } elseif ($LASTEXITCODE -ne 0) { throw "$Label grammar cache diff failed" }
    $revision = (& git -C $Cache rev-parse HEAD).Trim()
    if ($LASTEXITCODE) { throw "$Label grammar cache revision lookup failed" }
    $url = ([uri]($Cache.Replace('\','/') + '/')).AbsoluteUri.TrimEnd('/')
    return @($url, $revision)
}
$vexGrammar = Update-GrammarCache 'tree-sitter-vex' (Join-Path $Destination 'grammar') 'VEX'
$openclGrammar = Update-GrammarCache 'tree-sitter-opencl' (Join-Path $Destination 'grammar-opencl') 'OpenCL'
$extension = Join-Path $Destination 'zed-extension'
New-Item -ItemType Directory -Force $extension | Out-Null
foreach ($name in @('src', 'languages', 'server')) {
    $generatedSource = Join-Path $extension $name
    if (Test-Path $generatedSource) { Remove-Item $generatedSource -Recurse -Force }
}
foreach ($name in @('Cargo.toml','Cargo.lock','rust-toolchain.toml','build.rs','src','languages')) {
    Copy-Item (Join-Path $repo "editors/zed-houdini/$name") $extension -Recurse -Force
}
foreach ($package in @(@('tools/vex-lsp', 'ko_vex_lsp'), @('tools/apex-lsp', 'ko_apex_lsp'), @('tools/opencl-lsp', 'ko_opencl_lsp'))) {
    $server = Join-Path $extension ('server/' + $package[1])
    New-Item -ItemType Directory -Force $server | Out-Null
    Get-ChildItem (Join-Path $repo ($package[0] + '/' + $package[1])) -File -Filter '*.py' |
        Copy-Item -Destination $server -Force
}
$template = [IO.File]::ReadAllText((Join-Path $repo 'editors/zed-houdini/extension.toml.in'))
# JSON string quoting is also valid TOML basic-string quoting. Preserve the
# script exactly (including line endings), so capability checks match include_str!.
$discovery = [IO.File]::ReadAllText((Join-Path $extension 'src/discover_runtime.ps1')) | ConvertTo-Json -Compress
$manifest = $template.Replace('@GRAMMAR_REPOSITORY@', $vexGrammar[0]).Replace('@GRAMMAR_REV@', $vexGrammar[1]).Replace('@OPENCL_GRAMMAR_REPOSITORY@', $openclGrammar[0]).Replace('@OPENCL_GRAMMAR_REV@', $openclGrammar[1]).Replace('@DISCOVERY_SCRIPT@', $discovery)
[IO.File]::WriteAllText((Join-Path $extension 'extension.toml'), $manifest)
Write-Host "Prepared extension: $extension"
Write-Host 'In Zed: Extensions -> Install Dev Extension -> select that folder.'
Write-Host 'Then open VEX (.vfl), APEX Script (.apex) or Houdini OpenCL (.cl) in any workspace. No per-workspace setup is needed.'

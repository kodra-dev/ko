# Private Codex installation. Does not change global npm packages or Codex config.
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$root = if ($env:KO_AI_HOME) { $env:KO_AI_HOME } else { Join-Path $env:LOCALAPPDATA 'ko-ai' }
if (-not (Get-Command npm -ErrorAction SilentlyContinue)) {
    throw 'Install Node.js LTS (including npm), then rerun this script.'
}
New-Item -ItemType Directory -Force (Join-Path $root 'tools') | Out-Null
& npm install --prefix (Join-Path $root 'tools') '@openai/codex@0.168.0'
if ($LASTEXITCODE -ne 0) { throw 'Codex installation failed.' }
Write-Host "Installed the tested Codex version under $root/tools"
Write-Host 'In Houdini Python Shell: import ko_ai; ko_ai.reload()'
Write-Host 'Then click Connect and Sign in with ChatGPT. No Houdini restart needed.'

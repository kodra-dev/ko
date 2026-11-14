# Private Codex installation. No global npm/config changes or provider calls.
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
if ($env:LOOM_HOME) { $root = $env:LOOM_HOME }
elseif ($env:KO_AI_HOME -and ([IO.Path]::GetFullPath($env:KO_AI_HOME) -ne
        [IO.Path]::GetFullPath((Join-Path $env:LOCALAPPDATA "ko-ai")))) {
    $root = $env:KO_AI_HOME # Legacy custom location only; default paths migrate below.
}
else {
    $root = Join-Path $env:LOCALAPPDATA 'Loom'
    $legacy = Join-Path $env:LOCALAPPDATA 'ko-ai'
    if (Test-Path $legacy) {
        if (Test-Path $root) {
            if (Get-ChildItem -Force $root) { throw 'Both Loom and ko-ai data exist. Set LOOM_HOME explicitly; nothing was merged.' }
            Remove-Item $root
        }
        try {
            # Probe existing workspace handles before moving data; do not bypass live owners.
            foreach ($workspace in (Get-ChildItem (Join-Path $legacy 'workspaces') -Directory -ErrorAction SilentlyContinue)) {
                $lock = Join-Path $workspace.FullName 'workspace.lock'
                if (Test-Path $lock) {
                    $handle = [IO.File]::Open($lock, [IO.FileMode]::Open, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
                    $handle.Dispose()
                }
            }
            Move-Item $legacy $root
        }
        catch { throw 'Close all old Loom/Houdini and private Codex processes, then retry migration. No credential copying is performed.' }
    }
}
if (-not (Get-Command npm -ErrorAction SilentlyContinue)) {
    throw 'Install Node.js LTS (including npm), then rerun this script.'
}
New-Item -ItemType Directory -Force (Join-Path $root 'tools') | Out-Null
& npm install --prefix (Join-Path $root 'tools') '@openai/codex@0.168.0'
if ($LASTEXITCODE -ne 0) { throw 'Codex installation failed.' }
Write-Host "Installed the tested Codex version under $root/tools"
Write-Host 'In Houdini Python Shell: import loom; loom.open_pane()'
Write-Host 'Then Connect. Existing migrated ChatGPT login is retained; sign in only if needed.'

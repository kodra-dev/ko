use std::path::PathBuf;
use zed_extension_api::{self as zed, settings::LspSettings, serde_json::{self, Value}};
mod runtime;

struct Vex;
impl Vex {
    fn options(id: &zed::LanguageServerId, worktree: &zed::Worktree) -> zed::Result<Value> {
        let settings = LspSettings::for_worktree(id.as_ref(), worktree)?;
        runtime::options(settings.settings, settings.initialization_options)
    }
}

impl zed::Extension for Vex {
    fn new() -> Self { Self }

    fn language_server_command(&mut self, id: &zed::LanguageServerId, worktree: &zed::Worktree) -> zed::Result<zed::Command> {
        let settings = LspSettings::for_worktree(id.as_ref(), worktree)?;
        // Standard explicit binary override remains available, but is NOT required.
        if let Some(binary) = settings.binary {
            if let Some(command) = binary.path {
                return Ok(zed::Command { command, args: binary.arguments.unwrap_or_default(),
                    env: binary.env.unwrap_or_default().into_iter().collect() });
            }
        }
        let options = Self::options(id, worktree)?;
        let mut hfs = options.get("houdiniPath").and_then(Value::as_str).map(str::to_owned);
        // Compatibility only: old project files are optional, never generated.
        if hfs.is_none() {
            if let Ok(text) = worktree.read_text_file("vex-lsp.json") {
                if let Ok(legacy) = serde_json::from_str::<Value>(text.trim_start_matches('\u{feff}')) {
                    hfs = legacy.get("houdiniPath").and_then(Value::as_str).map(str::to_owned);
                }
            }
        }
        let mut env = Vec::new();
        let command = if let Some(python) = options.get("pythonPath").and_then(Value::as_str) {
            python.to_owned()
        } else if zed::current_platform().0 == zed::Os::Windows {
            let mut probe = zed::process::Command::new(runtime::DISCOVERY_PROGRAM)
                .args(runtime::DISCOVERY_ARGS.iter().copied());
            // Pass only discovery variables, not arbitrary shell environment/secrets.
            for (key, value) in worktree.shell_env() {
                if ["HFS", "ProgramFiles", "ProgramW6432"].iter().any(|wanted| key.eq_ignore_ascii_case(wanted)) {
                    probe = probe.env(key, value);
                }
            }
            if let Some(path) = &hfs {
                // Never interpolate a configured path into shell source.
                probe = probe.env("KO_VEX_EXPLICIT_HFS", path);
            }
            let result = probe.output()?;
            if result.status != Some(0) {
                return Err(format!("Houdini runtime discovery failed: {}", String::from_utf8_lossy(&result.stderr).trim()));
            }
            let discovered: Value = serde_json::from_slice(&result.stdout).map_err(|e| format!("Invalid Houdini discovery response: {e}"))?;
            hfs = discovered["houdiniPath"].as_str().map(str::to_owned);
            discovered["python"].as_str().ok_or("Houdini discovery did not return a Python interpreter")?.to_owned()
        } else {
            worktree.which("python3").or_else(|| worktree.which("python"))
                .ok_or("Python 3.10+ not found; set lsp.ko-vex.settings.pythonPath")?
        };
        if let Some(path) = hfs { env.push(("KO_VEX_HOUDINI".into(), path)); }
        // PWD is set by Zed's WASI host to this extension's writable work directory.
        let base = PathBuf::from(std::env::var("PWD").map_err(|_| "Zed extension work directory unavailable")?);
        let server = runtime::install_bundle(&base)?;
        env.push(("KO_VEX_CACHE".into(), base.join("docs").to_string_lossy().into_owned()));
        Ok(zed::Command { command, args: vec!["-I".into(), "-S".into(), "-c".into(), runtime::BOOTSTRAP.into(),
                         server.to_string_lossy().into_owned()], env })
    }

    fn language_server_initialization_options(&mut self, id: &zed::LanguageServerId, worktree: &zed::Worktree) -> zed::Result<Option<Value>> {
        Ok(Some(Self::options(id, worktree)?))
    }

    fn language_server_workspace_configuration(&mut self, id: &zed::LanguageServerId, worktree: &zed::Worktree) -> zed::Result<Option<Value>> {
        Ok(Some(serde_json::json!({"ko-vex": Self::options(id, worktree)?})))
    }
}
zed::register_extension!(Vex);

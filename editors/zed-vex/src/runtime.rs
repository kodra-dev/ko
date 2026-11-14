use std::{fs, path::{Path, PathBuf}};
use zed_extension_api::serde_json::{self, Value};

include!(concat!(env!("OUT_DIR"), "/server_bundle.rs"));

pub const BOOTSTRAP: &str = "import sys,runpy; assert sys.version_info >= (3,10), 'Ko VEX requires Python 3.10+'; sys.path.insert(0,sys.argv[1]); runpy.run_module('ko_vex_lsp',run_name='__main__')";

pub fn options(settings: Option<Value>, initialization: Option<Value>) -> Result<Value, String> {
    let mut result = serde_json::Map::new();
    for value in [settings, initialization].into_iter().flatten() {
        let object = value.as_object().ok_or("Ko VEX settings must be a JSON object")?;
        result.extend(object.clone());
    }
    Ok(Value::Object(result))
}

pub fn install_bundle(base: &Path) -> Result<PathBuf, String> {
    let directory = base.join(format!("server-{BUNDLE_ID}"));
    for (relative, source) in SERVER_FILES {
        let path = directory.join(relative);
        fs::create_dir_all(path.parent().unwrap()).map_err(|e| e.to_string())?;
        // Repair missing/corrupted files; never rely solely on a version marker.
        if fs::read_to_string(&path).ok().as_deref() != Some(source) {
            fs::write(&path, source).map_err(|e| e.to_string())?;
        }
    }
    Ok(directory)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn normal_settings_and_legacy_initialization_merge() {
        let merged = options(Some(serde_json::json!({"includePaths":["custom"],"context":"cvex"})),
                             Some(serde_json::json!({"context":"sop"}))).unwrap();
        assert_eq!(merged["context"], "sop");
        assert_eq!(merged["includePaths"][0], "custom");
        assert!(options(Some(serde_json::json!(false)), None).is_err());
    }
    #[test]
    fn bundled_houdini_python_runs_embedded_lsp_without_project_setup() {
        use std::{io::Write, process::{Command, Stdio}};
        let Ok(hfs) = std::env::var("KO_VEX_TEST_HFS") else { return; };
        let base = std::env::temp_dir().join(format!("ko-vex-lsp-smoke-{}", std::process::id()));
        let directory = install_bundle(&base).unwrap();
        let python = Path::new(&hfs).join("python313/python.exe");
        let mut command = Command::new(python);
        command.args(["-I", "-S", "-c", BOOTSTRAP]).arg(&directory)
            .stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::piped());
        #[cfg(windows)] {
            use std::os::windows::process::CommandExt;
            command.creation_flags(0x08000000);
        }
        let mut child = command.spawn().unwrap();
        let body = serde_json::json!({"jsonrpc":"2.0", "id":1, "method":"initialize", "params":{
            "rootUri":format!("file:///{}", base.to_string_lossy().replace('\\', "/")),
            "initializationOptions":{"discoverHoudini":false}}}).to_string();
        write!(child.stdin.take().unwrap(), "Content-Length: {}\r\n\r\n{}", body.len(), body).unwrap();
        let result = child.wait_with_output().unwrap();
        assert!(result.status.success(), "{}", String::from_utf8_lossy(&result.stderr));
        assert!(String::from_utf8_lossy(&result.stdout).contains("ko-vex-lsp"));
        assert!(!base.join("vex-lsp.json").exists());
        assert!(!base.join(".zed").exists());
        fs::remove_dir_all(base).unwrap();
    }
    #[test]
    fn bundle_is_self_contained_and_repairs_files() {
        let base = std::env::temp_dir().join(format!("ko-vex-rust-{}", std::process::id()));
        let directory = install_bundle(&base).unwrap();
        assert!(directory.join("ko_vex_lsp/__main__.py").is_file());
        assert!(directory.join("ko_vex_lsp/discovery.py").is_file());
        let target = directory.join("ko_vex_lsp/model.py");
        fs::write(&target, "damaged").unwrap();
        assert_eq!(install_bundle(&base).unwrap(), directory);
        assert_ne!(fs::read_to_string(target).unwrap(), "damaged");
        fs::remove_dir_all(base).unwrap();
    }
}

use std::{fs, path::{Path, PathBuf}};
use zed_extension_api::serde_json::{self, Value};

include!(concat!(env!("OUT_DIR"), "/server_bundle.rs"));

// Keep the executable name unqualified: Zed matches process capabilities against
// the requested command literally (not the basename of an absolute PATH result).
pub const DISCOVERY_PROGRAM: &str = "powershell.exe";
pub const DISCOVERY_ARGS: &[&str] = &[
    "-NoLogo", "-NoProfile", "-NonInteractive", "-WindowStyle", "Hidden", "-Command",
    include_str!("discover_runtime.ps1"),
];

// argv[1] is the bundle directory, argv[2] the server package to run.
pub const BOOTSTRAP: &str = "import sys,runpy; assert sys.version_info >= (3,10), 'Ko Houdini language servers require Python 3.10+'; sys.path.insert(0,sys.argv[1]); runpy.run_module(sys.argv[2],run_name='__main__')";

pub const VEX_SERVER: &str = "ko-vex";
pub const APEX_SERVER: &str = "ko-apex";
pub const OPENCL_SERVER: &str = "ko-opencl";
// Settings a server inherits from lsp.ko-vex when its own settings omit them.
pub const SHARED_SETTINGS: &[&str] = &["houdiniPath", "pythonPath", "discoverHoudini"];

pub fn server_module(id: &str) -> Option<&'static str> {
    match id {
        VEX_SERVER => Some("ko_vex_lsp"),
        APEX_SERVER => Some("ko_apex_lsp"),
        OPENCL_SERVER => Some("ko_opencl_lsp"),
        _ => None,
    }
}

pub fn options(settings: Option<Value>, initialization: Option<Value>) -> Result<Value, String> {
    let mut result = serde_json::Map::new();
    for value in [settings, initialization].into_iter().flatten() {
        let object = value.as_object().ok_or("Ko language server settings must be a JSON object")?;
        result.extend(object.clone());
    }
    Ok(Value::Object(result))
}

pub fn inherit(shared: &Value, own: Value) -> Value {
    let mut result = own.as_object().cloned().unwrap_or_default();
    for key in SHARED_SETTINGS {
        if let Some(value) = shared.get(*key) {
            result.entry(key.to_string()).or_insert_with(|| value.clone());
        }
    }
    Value::Object(result)
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
    fn apex_inherits_only_shared_vex_settings() {
        let shared = serde_json::json!({"houdiniPath":"C:/H", "pythonPath":"C:/P/python.exe", "context":"sop"});
        let merged = inherit(&shared, serde_json::json!({"pythonPath":"own"}));
        assert_eq!(merged["houdiniPath"], "C:/H");
        assert_eq!(merged["pythonPath"], "own");
        assert!(merged.get("context").is_none());
        assert_eq!(server_module("ko-apex"), Some("ko_apex_lsp"));
        assert_eq!(server_module("ko-vex"), Some("ko_vex_lsp"));
        assert_eq!(server_module("ko-opencl"), Some("ko_opencl_lsp"));
        assert_eq!(server_module("other"), None);
    }
    #[test]
    fn bundled_houdini_python_runs_embedded_lsp_without_project_setup() {
        use std::{io::Write, process::{Command, Stdio}};
        let Ok(hfs) = std::env::var("KO_VEX_TEST_HFS") else { return; };
        let base = std::env::temp_dir().join(format!("ko-vex-lsp-smoke-{}", std::process::id()));
        let directory = install_bundle(&base).unwrap();
        let python = Path::new(&hfs).join("python313/python.exe");
        let mut command = Command::new(python);
        command.args(["-I", "-S", "-c", BOOTSTRAP]).arg(&directory).arg("ko_vex_lsp")
            .env("KO_VEX_CACHE", base.join("docs"))
            .stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::piped());
        #[cfg(windows)] {
            use std::os::windows::process::CommandExt;
            command.creation_flags(0x08000000);
        }
        let mut child = command.spawn().unwrap();
        let root_uri = format!("file:///{}", base.to_string_lossy().replace('\\', "/"));
        let uri = format!("{root_uri}/case1.vfl");
        let messages = [
            serde_json::json!({"jsonrpc":"2.0", "id":1, "method":"initialize", "params":{
                "rootUri":root_uri, "initializationOptions":{"houdiniPath":hfs}}}),
            serde_json::json!({"jsonrpc":"2.0", "method":"textDocument/didOpen", "params":{
                "textDocument":{"uri":uri,"version":1,"text":"float value = fit(0.5, 0, 1, 0, 10);"}}}),
            serde_json::json!({"jsonrpc":"2.0", "id":2,"method":"textDocument/completion","params":{
                "textDocument":{"uri":uri},"position":{"line":0,"character":16}}}),
            serde_json::json!({"jsonrpc":"2.0", "id":3,"method":"textDocument/hover","params":{
                "textDocument":{"uri":uri},"position":{"line":0,"character":15}}}),
        ];
        let mut input = child.stdin.take().unwrap();
        for message in messages {
            let body = message.to_string();
            write!(input, "Content-Length: {}\r\n\r\n{}", body.len(), body).unwrap();
        }
        drop(input);
        let result = child.wait_with_output().unwrap();
        assert!(result.status.success(), "{}", String::from_utf8_lossy(&result.stderr));
        let output = String::from_utf8_lossy(&result.stdout);
        assert!(output.contains("ko-vex-lsp"));
        assert!(output.contains("\"label\": \"fit\""), "{output}");
        assert!(output.contains("SideFX documentation"), "{output}");
        assert!(!output.contains("\"error\":"), "{output}");
        assert!(!base.join("vex-lsp.json").exists());
        assert!(!base.join(".zed").exists());
        fs::remove_dir_all(base).unwrap();
    }
    #[test]
    fn bundled_houdini_python_runs_embedded_apex_lsp() {
        use std::{io::Write, process::{Command, Stdio}};
        let Ok(hfs) = std::env::var("KO_VEX_TEST_HFS") else { return; };
        let base = std::env::temp_dir().join(format!("ko-apex-lsp-smoke-{}", std::process::id()));
        let directory = install_bundle(&base).unwrap();
        let mut command = Command::new(Path::new(&hfs).join("python313/python.exe"));
        command.args(["-I", "-S", "-c", BOOTSTRAP]).arg(&directory).arg("ko_apex_lsp")
            .env("KO_VEX_CACHE", base.join("docs"))
            .stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::piped());
        #[cfg(windows)] {
            use std::os::windows::process::CommandExt;
            command.creation_flags(0x08000000);
        }
        let mut child = command.spawn().unwrap();
        let root_uri = format!("file:///{}", base.to_string_lossy().replace('\\', "/"));
        let uri = format!("{root_uri}/case.apex");
        let messages = [
            serde_json::json!({"jsonrpc":"2.0", "id":1, "method":"initialize", "params":{
                "rootUri":root_uri, "initializationOptions":{"houdiniPath":hfs}}}),
            serde_json::json!({"jsonrpc":"2.0", "method":"textDocument/didOpen", "params":{
                "textDocument":{"uri":uri,"version":1,"text":"x = sine(1.0)\nwhile True:\n    pass\n"}}}),
            serde_json::json!({"jsonrpc":"2.0", "id":2,"method":"textDocument/hover","params":{
                "textDocument":{"uri":uri},"position":{"line":0,"character":5}}}),
        ];
        let mut input = child.stdin.take().unwrap();
        for message in messages {
            let body = message.to_string();
            write!(input, "Content-Length: {}\r\n\r\n{}", body.len(), body).unwrap();
        }
        drop(input);
        let result = child.wait_with_output().unwrap();
        assert!(result.status.success(), "{}", String::from_utf8_lossy(&result.stderr));
        let output = String::from_utf8_lossy(&result.stdout);
        assert!(output.contains("ko-apex-lsp"), "{output}");
        assert!(output.contains("While is not supported in Apex Script."), "{output}");
        assert!(output.contains("sine(radians: Float) -> Float"), "{output}");
        assert!(!output.contains("\"error\":"), "{output}");
        fs::remove_dir_all(base).unwrap();
    }
    #[test]
    fn bundled_houdini_python_runs_embedded_opencl_lsp() {
        use std::{io::Write, process::{Command, Stdio}};
        let Ok(hfs) = std::env::var("KO_VEX_TEST_HFS") else { return; };
        let base = std::env::temp_dir().join(format!("ko-opencl-lsp-smoke-{}", std::process::id()));
        let directory = install_bundle(&base).unwrap();
        let mut command = Command::new(Path::new(&hfs).join("python313/python.exe"));
        command.args(["-I", "-S", "-c", BOOTSTRAP]).arg(&directory).arg("ko_opencl_lsp")
            .stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::piped());
        #[cfg(windows)] {
            use std::os::windows::process::CommandExt;
            command.creation_flags(0x08000000);
        }
        let mut child = command.spawn().unwrap();
        let root_uri = format!("file:///{}", base.to_string_lossy().replace('\\', "/"));
        let uri = format!("{root_uri}/case.cl");
        let text = "#bind point &P float3\n#bind point mass flot\n@KERNEL\n{\n    mat3identity(m);\n}\n";
        let messages = [
            serde_json::json!({"jsonrpc":"2.0", "id":1, "method":"initialize", "params":{
                "rootUri":root_uri, "initializationOptions":{"houdiniPath":hfs}}}),
            serde_json::json!({"jsonrpc":"2.0", "method":"textDocument/didOpen", "params":{
                "textDocument":{"uri":uri,"version":1,"text":text}}}),
            serde_json::json!({"jsonrpc":"2.0", "id":2,"method":"textDocument/hover","params":{
                "textDocument":{"uri":uri},"position":{"line":4,"character":6}}}),
        ];
        let mut input = child.stdin.take().unwrap();
        for message in messages {
            let body = message.to_string();
            write!(input, "Content-Length: {}\r\n\r\n{}", body.len(), body).unwrap();
        }
        drop(input);
        let result = child.wait_with_output().unwrap();
        assert!(result.status.success(), "{}", String::from_utf8_lossy(&result.stderr));
        let output = String::from_utf8_lossy(&result.stdout);
        assert!(output.contains("ko-opencl-lsp"), "{output}");
        assert!(output.contains("is not a documented #bind flag or type"), "{output}");
        assert!(output.contains("mat3identity(mat3 a)"), "{output}");
        assert!(!output.contains("\"error\":"), "{output}");
        fs::remove_dir_all(base).unwrap();
    }
    #[test]
    fn bundle_is_self_contained_and_repairs_files() {
        let base = std::env::temp_dir().join(format!("ko-vex-rust-{}", std::process::id()));
        let directory = install_bundle(&base).unwrap();
        assert!(directory.join("ko_vex_lsp/__main__.py").is_file());
        assert!(directory.join("ko_vex_lsp/discovery.py").is_file());
        assert!(directory.join("ko_apex_lsp/__main__.py").is_file());
        assert!(directory.join("ko_apex_lsp/catalog.py").is_file());
        assert!(directory.join("ko_opencl_lsp/__main__.py").is_file());
        assert!(directory.join("ko_opencl_lsp/builtins.py").is_file());
        let target = directory.join("ko_vex_lsp/model.py");
        fs::write(&target, "damaged").unwrap();
        assert_eq!(install_bundle(&base).unwrap(), directory);
        assert_ne!(fs::read_to_string(target).unwrap(), "damaged");
        fs::remove_dir_all(base).unwrap();
    }
}

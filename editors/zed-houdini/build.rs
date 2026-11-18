//! Embed the server at extension-build time. No runtime path into this checkout.
use std::{env, fs, path::{Path, PathBuf}};

fn collect(root: &Path, directory: &Path, files: &mut Vec<(String, String)>) {
    for entry in fs::read_dir(directory).expect("read server sources") {
        let path = entry.expect("server source entry").path();
        if path.is_dir() {
            if path.file_name().unwrap() != "__pycache__" { collect(root, &path, files); }
        } else if path.extension().is_some_and(|ext| ext == "py") {
            println!("cargo:rerun-if-changed={}", path.display());
            files.push((path.strip_prefix(root).unwrap().to_string_lossy().replace('\\', "/"),
                        fs::read_to_string(&path).expect("UTF-8 server source")));
        }
    }
}

fn main() {
    let crate_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let bundled = crate_dir.join("server");
    // (root, package): a packaged extension has server/<package>; a checkout has tools/*-lsp.
    let packages = if bundled.is_dir() {
        vec![(bundled.clone(), "ko_vex_lsp"), (bundled.clone(), "ko_apex_lsp"), (bundled, "ko_opencl_lsp")]
    } else {
        vec![(crate_dir.join("../../tools/vex-lsp"), "ko_vex_lsp"), (crate_dir.join("../../tools/apex-lsp"), "ko_apex_lsp"),
             (crate_dir.join("../../tools/opencl-lsp"), "ko_opencl_lsp")]
    };
    let mut files = Vec::new();
    // Only the runtime packages are embedded, not packaging scripts or tests.
    for (root, package) in &packages {
        println!("cargo:rerun-if-changed={}", root.display());
        collect(root, &root.join(package), &mut files);
    }
    files.sort();
    let mut hash = 0xcbf29ce484222325u64;
    for (path, content) in &files {
        for byte in path.bytes().chain([0]).chain(content.bytes()).chain([0]) {
            hash = (hash ^ byte as u64).wrapping_mul(0x100000001b3);
        }
    }
    let mut generated = format!("pub const BUNDLE_ID: &str = \"{hash:016x}\";\npub const SERVER_FILES: &[(&str, &str)] = &[\n");
    for (path, content) in files { generated.push_str(&format!("({path:?}, {content:?}),\n")); }
    generated.push_str("];\n");
    fs::write(PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("server_bundle.rs"), generated).unwrap();
}

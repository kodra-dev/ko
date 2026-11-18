# VEX support in Zed — initial standalone implementation

This is independent of `ko_zed`: Houdini does **not** have to be running.
It uses an installed Houdini's `vcc` and documentation, not an AI/provider.
No authentication, global editor settings, or scene cooking is involved.

## Available now

- A native VEX Tree-sitter grammar: highlighting, indentation, bracket matching,
  function/struct outline, and Vim text objects. Recognizes wrangle attributes,
  array types, structs, methods, preprocessing, and VEX parameter separators.
- Built-in function completion and overload/signature help from the **configured
  installation's compiler**, rather than a bundled, potentially obsolete API list.
- Hover documentation from that installation's `houdini/help/vex.zip`, plus links
  to SideFX's online documentation. Completion documentation is fetched lazily.
- Local-variable/parameter completion and go-to-definition.
- Include-path completion and navigation into literal `#include <...>` / `"..."`.
- Transitive header indexing: functions, structs, globals, macros, and comments.
  Supports official package headers such as `kinefx.h`, and custom headers such
  as this repository's `vex/include/ko_capture.h`.
- Header-defined struct field/method completion, hover and navigation for simple
  typed receivers; vector component completion; known/source-declared attribute
  completion. No geometry inspection is needed.
- Unsaved header buffers take precedence over disk; changed disk headers are
  reindexed lazily. Cycles and graph/file-size limits are handled.
- **Compiler-gated local variable and parameter rename and find-references.**
  See the safety boundary below.
- Delimiter/missing-literal-header diagnostics while editing. Optional actual
  compiler errors on save for standalone source mode.

This is a useful first version, **not a complete VEX compiler frontend or a
claim of full IDE parity**. In particular, browsing an included declaration is
not the same as proving a particular overload/conditional branch is active.

## Install once — no workspace setup

The prepared local extension is at:

```text
build/zed-vex/zed-extension
```

That path is relative to this repository and is generated/gitignored. Regenerate
it from the repository root with:

```powershell
./scripts/vex_package.ps1
```

In Zed: **Extensions → Install Dev Extension**, select that folder. Then open
VEX in **any workspace**, including the existing Houdini–Zed slot folders.
You do not run a setup script, start a server, install Python separately, or
create `vex-lsp.json` / per-workspace `.zed/settings.json`.

This is still an **unpublished local extension**, not an entry in Zed's public
registry. Development installation needs Rust/rustup and Git; Zed builds the
extension and grammar as it does for other dev extensions. The prepared package
is independent of the source checkout, but its local grammar cache must remain
at its prepared location. No extension has been published or uploaded.

At first VEX language-server startup on Windows the extension:

1. Finds the selected Houdini installation (an explicit override, inherited
   `HFS`, or the highest numerically versioned supported standard installation).
2. Uses Houdini's **regular bundled Python 3.10+**, never `hython`, `hou` or a
   license-consuming Houdini application. No additional Python installation.
3. Materializes its embedded server in Zed's extension work directory and starts
   it automatically. Nothing is loaded from this repository at runtime.
4. Keeps generated documentation in its cache, not in each project.

The one-time discovery probe is a hidden, no-profile PowerShell process. It is
not a shell proxy for the LSP connection: Zed launches Python directly afterwards.
The generated manifest grants `process:exec` only for the exact fixed discovery
command and arguments, not arbitrary shell commands. After regenerating a package
with manifest/startup changes, **rebuild or reinstall the dev extension** in Zed;
restarting only the language server does not reload its extension permissions.
Startup diagnostics identify the chosen Houdini installation; with multiple
versions, use the override below if the newest is not the one you want.

On Linux/macOS the server also discovers standard Houdini paths, but extension
startup currently needs `python3` on PATH or the optional `pythonPath` setting.
Automatic use of Houdini's bundled interpreter is currently implemented/tested
on Windows.

`.vfl` and `.vexh` are recognized automatically. For `.h` files, select **VEX**
in Zed's language selector. We deliberately do not steal every C/C++ header.
The server indexes includes regardless of their editor language; unsaved header
changes require that header to be attached to the **same VEX server instance**.
Cross-worktree delivery of external header buffers is still pending manual Zed
acceptance. Project-specific associations can use Zed's `file_types`.

If syntax highlighting works but `fi` completion / `fit` hover do not, check
`zed: open log` for a `ko-vex` startup error. Highlighting and word-based
suggestions can work without the VEX language server. A `process:exec ... was not
listed in the extension manifest` error means an outdated/missing startup
permission; regenerate the package and rebuild/reinstall the extension, rather
than broadening Zed's global permissions.

## Optional normal Zed settings

No settings are required for the standard Windows installation. Overrides belong
in normal Zed user settings (apply everywhere) or project settings (if desired):

```json
{
  "lsp": {
    "ko-vex": {
      "settings": {
        "houdiniPath": "C:/Program Files/Side Effects Software/Houdini 22.0.466",
        "includePaths": [
          "H:/Assets/Assets_3D/HoudiniScripts/ko/vex/include"
        ],
        "context": "cvex",
        "mode": "snippet",
        "compilerTimeout": 8,
        "compilerDiagnostics": false
      }
    }
  }
}
```

Leave `houdiniPath` unset for automatic discovery. For a custom Python runtime,
set `lsp.ko-vex.settings.pythonPath` to its absolute executable path; it must be
Python 3.10+. A standard `lsp.ko-vex.binary` override still replaces the entire
server command, for advanced/debug use only. Neither is required normally.

Include paths are **include directories**, not Houdini package roots. Relative
paths are workspace-relative. Quoted includes first search the including file's
directory. Angle includes search:

1. Explicit `includePaths`, in order.
2. Project-local `vex/include` and `include` directories.
3. Already-expanded `HOUDINI_PATH` entries' `vex/include` directories.
4. Installed `$HFS/packages/*/vex/include` directories (deterministic fallback).
5. `$HFS/houdini/vex/include`.

This does not execute Houdini package configuration/scripts or interpret special
Houdini path tokens. Installed package discovery can include inactive packages.
For exact session-specific selection/precedence, list its effective directories
in `includePaths`. `HOUDINI_VEX_PATH` is not an include-path variable here.

`mode: "snippet"` supports wrangle-style statement buffers. Local rename
validation uses a temporary CVEX wrapper and same-width attribute substitutions.
Explicit attribute prefixes and known standard bindings are supported; unknown
types cause refusal. This is not a faithful reproduction of every generated
Houdini kernel.

Use `mode: "source"` for standalone VEX compilation units. With
`compilerDiagnostics: true`, saves report actual compiler errors and header
locations. Compilation is not triggered on each keystroke, and diagnostics are
not enabled for approximate wrangle wrappers.

Restart the language server after changing the installation or Python override.
Old `vex-lsp.json` files and `initialization_options` remain compatible, but are
not created or required. Zed options override old project-file values.

## Developer packaging (not user setup)

Run `./scripts/vex_package.ps1` from the repository root to regenerate the
installable **development artifact** under `build/zed-vex/`. The entire generated
directory, including its grammar cache and any Zed build outputs, is gitignored;
only sources and the packaging script belong in version control. An optional
`-Destination` override can place output outside the repository.

It embeds the server at Rust build time, copies all sources needed to build the
extension without this checkout, and prepares an isolated local grammar Git cache.
It does not touch any workspace, global settings, authentication, or source-repo
history. Repackage after source changes, then rebuild/reinstall the dev extension.

The old `vex_setup.ps1` per-workspace bootstrap has been removed. Publishing a
portable public-registry extension, including hosted grammar sources, is a
separate release task; this change does not pretend that publication happened.

## Dev-install troubleshooting: old default Rust

If Zed reports `compiling Rust extension` and its log says it cannot find the
`wasm32-wasip2` target specification, its **default Rust toolchain is too old**.
Zed runs this preflight before building in the extension directory, so the
extension's `rust-toolchain.toml` does not fix that particular check.

If the pinned toolchain is already installed, fully quit Zed, then launch it
from PowerShell at this repository root with:

```powershell
rustup run 1.99.1 "$env:LOCALAPPDATA/Programs/Zed/bin/zed.exe" .
```

Then retry **Install Dev Extension → `build/zed-vex/zed-extension`**. The
`rustup run` override applies only to that Zed process; it does not change your
global Rust default. Fully quitting matters: a CLI request routed to an already
running Zed instance does not update that instance's environment.

Check toolchain availability with `rustup toolchain list`. If absent, install it
with `rustup toolchain install 1.99.1 --profile minimal --target wasm32-wasip2`.
The detailed install error is available through **zed: open log**.

## Rename / references safety boundary

Currently supported: **document-local variables and function parameters**, with
braced lexical scopes, including braced `for`/`foreach` loops and shadowing.

Before offering rename/references, the server:

- Requires a complete, bounded literal-include graph.
- Compiles an isolated snapshot, including unsaved headers, without running it.
- Checks compiler declaration/reference locations and lexical scope bindings
  against the source index. It refuses mismatched/ambiguous mappings.

Before returning rename edits, it also:

- Rejects keyword/invalid names and existing identifiers/macros/built-ins that
  could cause capture or ambiguity. This is intentionally stricter than necessary.
- Compiles the renamed snapshot and compares its AST structure.
- Returns versioned LSP edits; it never edits source files itself.

Comments, strings, member names and geometry attribute names are not local-variable
references. Renaming `weight` does not rename `f@weight` or `"weight"`.
Houdini installation files are read-only for refactoring.

**Refused/not implemented:** function/struct/member/global/attribute/macro rename,
externally bound shader parameters, unbraced loop scopes, documents declaring
structs, document preprocessing beyond literal includes, macro invocation in the
main document, macro-generated includes, unresolved headers, and unsupported
compiler/source mappings. A header can use ordinary include guards, but macro
calls in the main document conservatively disable rename.

References share this same local-variable scope; there is no claim that every
HIP/HDA or unopened workspace file has been searched.

## Other current limits / follow-up work

- Header browsing indexes conditional branches as **candidates**; it does not
  implement full preprocessing or overload/type resolution. Go-to-definition may
  show multiple locations. Compiler-gated refactors do not rely on these guesses.
- Struct inference covers simple locals, attributes, field chains and array
  indexing. It is not general expression inference or a complete method system.
- No formatter, workspace-wide symbol/refactor engine, or scene-aware bindings.
- No bridge dependency or bridge changes in this implementation.
- Requests are processed serially. Compiler operations have per-process timeouts
  but are not asynchronously cancelled; cancellation/worker scheduling is future
  work. Large compiler requests can temporarily delay other language requests.
- Bounds: 4 MiB per file, 256 indexed files, 16 MiB per include graph.
- Syntax grammar/corpus and Rust/WASM builds are tested, but actual extension
  installation and feature behavior inside Zed still need manual acceptance.

## Tests

Default offline suite (no compiler, Houdini, Zed, network or provider required):

```powershell
python -m unittest discover -s tests/ko -v
```

Local compiler smoke (no GUI, `hou`, scene, provider, or subscription use):

```powershell
$env:KO_VEX_TEST_HFS = 'C:/Program Files/Side Effects Software/Houdini 22.0.466'
python -m unittest discover -s tests/ko -p test_vex_lsp.py -v
Remove-Item Env:KO_VEX_TEST_HFS
```

Grammar development (first `npm ci` downloads the pinned CLI; subsequent tests
are offline):

```powershell
cd editors/tree-sitter-vex
npm ci
npm run generate
npm test
```

Extension build:

```powershell
cd editors/zed-vex
cargo build --locked --target wasm32-wasip2
```

Manual Zed acceptance:

1. Install the prepared dev extension, open any project and a `.vfl` file with no project configuration. Verify automatic startup and the chosen Houdini installation in the language-server log.
2. Type `fit(`; verify overloads, argument help and hover documentation.
3. Include `<kinefx.h>` and `<ko_capture.h>`; complete/navigate into their functions.
4. Open a custom header as VEX, change it without saving, and verify its new
   declarations/members appear in the including file's completion/navigation.
5. Rename a local in a nested shadowing example; inspect the edit preview. Ensure
   strings, comments and same-named attributes remain unchanged.
6. Try a collision, macro invocation and function rename; verify safe refusal.
7. Verify source-mode compiler errors after saving invalid standalone code.
8. Verify unrelated Python/C/C++ projects and global Zed settings are unchanged.

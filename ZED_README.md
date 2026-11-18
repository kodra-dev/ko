# Houdini code in Zed

Edit Houdini parameters and shelf tools in [Zed](https://zed.dev), with language support for VEX,
APEX Script, Houdini OpenCL and Houdini Python. Windows, Houdini 22.

## Setup

**1. Load this repo in Houdini** with a package file in your Houdini preferences folder, e.g.
`C:/Users/<you>/houdini22.0/packages/ko.json`:

```json
{
  "enable": true,
  "hpath": "H:/Assets/Assets_3D/HoudiniScripts/ko",
  "env": [
    { "var": "KO", "value": "H:/Assets/Assets_3D/HoudiniScripts/ko" },
    { "var": "HOUDINI_MENU_PATH", "value": "$KO/menu_overrides;&" }
  ]
}
```

The `HOUDINI_MENU_PATH` line hides SideFX Labs' duplicate **Edit in External Editor** menu items.
Remove it if Labs is disabled, or Houdini reports "Could not find element" at startup.
Restart Houdini after installing or updating the repo.

**2. Install the Zed extension** (VEX, APEX Script and OpenCL support):

```powershell
./scripts/houdini_package.ps1
```

Then in Zed: **Extensions → Install Dev Extension** and select `build/zed-houdini/zed-extension`.
It needs Rust (rustup) and Git, which Zed uses to build it. It finds your Houdini installation by
itself; there is nothing to configure per project. If an older **Houdini VEX** dev extension is
installed, uninstall it first.

After pulling changes to the extension, run the script again, then rebuild the dev extension in
Zed's Extensions page. Restarting the language server alone doesn't load a new extension version.

## Editing code from Houdini

- Right-click a code or string **parameter label → Edit in Zed**, or click into the field and press
  **Alt+E**. Right-click a **shelf tool → Edit in Zed** edits its script.
- **Save in Zed** to apply. The parameter updates within a second; keep the tab open.
- Everything opens in one dedicated Zed window (`Houdini-Zed-01`; a second Houdini gets `02`). Keep
  it for Houdini code only. Fold its side docks once and close it normally; Zed then remembers that
  layout.
- **Ko → Zed → Zed Sessions…** lists open edits: apply manually, reload from Houdini, toggle
  auto-apply, or detach.
- **Ko → Zed → Sync Houdini → Zed** refreshes all open files from their parameters (changed files
  are backed up first).

Good to know:

- **Auto-apply is on**: every save cooks the node, and Python code runs. Turn it off per file in
  Zed Sessions, or use Houdini's manual update mode.
- If both Houdini and Zed changed the same code, auto-apply pauses; resolve it in Zed Sessions.
- Each applied save is one Houdini undo step. The code is stored in the parameter, so save the HIP
  as usual.
- Only literal strings can be edited, not expressions, keyframed or locked parameters.
- Loading another HIP detaches everything; reopen what you need.
- Keep **Preferences → Scripting → Use External Editor** off, or Alt+E won't work. Set
  `KO_ZED_SHORTCUTS=0` before starting Houdini to disable the Alt+E/Ctrl+E handling.

## Language support

| File | Language | You get |
|---|---|---|
| `.vfl`, `.vexh` | VEX | Completion, hover docs, signature help, go to definition, live compiler errors and warnings, safe rename of local variables |
| `.apex` | APEX Script | Completion, hover docs, signature help, go to definition, syntax errors and unsupported constructs (`while`, `class`, …) |
| `.py` | Python | `hou` completion, hover and type checking, through Zed's built-in Python support |
| `.cl` | Houdini OpenCL | Completion and hover for `@` bindings and their methods, `#bind` types and flags, Houdini's OpenCL headers and OpenCL built-ins; signature help, go to definition, `#bind` and bracket checks |

**VEX** uses your Houdini's own compiler (`vcc`) and help, so built-ins and errors match your
version. Wrangle snippets are checked as snippets. Use `v@name`-style prefixes, or set
`attributeTypes` (below), so custom attributes have a known type. To edit a `.h` header as VEX,
pick VEX in Zed's language selector. Headers are found in `vex/include` folders on `HOUDINI_PATH`,
in Houdini's packages, and in `includePaths`.

**APEX Script** reads Houdini's own list of APEX functions. It can't report undefined names or
wrong argument types, because header templates and saved subgraphs define names it doesn't see;
Houdini's node errors remain the reference.

**OpenCL** understands Houdini's additions: `#bind` lines, `@KERNEL`/`@WRITEBACK` blocks, and `@name`
bindings with the methods their kind allows (`@P.set()`, `@surface.worldSample()`, `@ramp(0.5)`, …).
Headers come from `$HFS/houdini/ocl/include`; `matrix.h` is available without an include, as in
Houdini's snippets. There is **no compile check**: Houdini generates the real kernel from the `@`
syntax inside the node, so compile errors still appear only on the node. An `@name` with no `#bind`
is not flagged, because the node's Bindings tab can declare it. Words in `#bind` lines that the help
page doesn't document are shown as information, not errors. Houdini's own `.h` headers stay C/C++ in
Zed; pick **Houdini OpenCL** in the language selector to edit one with these features.

**Python** needs no setup for code opened from Houdini. For a folder you open in Zed yourself
(such as a tools repo), run once:

```powershell
python python3.13libs/ko_hou_stubs.py "C:/Program Files/Side Effects Software/Houdini 22.0.466" --project .
```

Add `--implicit-hou` if the folder holds snippets or callbacks that use `hou`/`kwargs` without
importing them. After settings change, run **editor: restart language server** in Zed.
Limitations: `hou.Node` results accept any node type's methods, values without type information
(such as `parm.eval()`) are `Any`, `apex` has no stubs, and for Qt completion import from `PySide6`
instead of `hutil.Qt`.

## Upgrading Houdini

- **Python:** the first **Edit in Zed** in the new version generates its `hou` stub under
  `editors/houdini-python/stubs/<version>/`; commit that folder. For folders set up with
  `ko_hou_stubs.py --project`, run the command again with the new Houdini path.
- **VEX and APEX:** the extension uses the newest installed Houdini. Restart the language servers in
  Zed (**editor: restart language server**). If you keep several versions and want an older one, set
  `houdiniPath` (below).
- **Alt+E** relies on Houdini UI internals. If it stops working after an upgrade, use the right-click
  menu and set `KO_ZED_SHORTCUTS=0`.

## Settings

Environment variables, set before starting Houdini:

- `KO_ZED_EXE`: path to Zed's `zed.exe` CLI, if it isn't on `PATH` or in the standard install location.
- `KO_ZED_WORKSPACE`: where the Houdini Zed workspaces live (default `$HOUDINI_USER_PREF_DIR/zed-editing`).
- `KO_ZED_SHORTCUTS=0`: disable the Alt+E/Ctrl+E field shortcuts.

Zed settings (all optional), in your Zed user settings:

```json
{
  "lsp": {
    "ko-vex": {
      "settings": {
        "houdiniPath": "C:/Program Files/Side Effects Software/Houdini 22.0.466",
        "includePaths": ["H:/Assets/Assets_3D/HoudiniScripts/ko/vex/include"],
        "attributeTypes": { "custom_velocity": "vector" },
        "compilerDiagnostics": true
      }
    }
  }
}
```

`houdiniPath` and `pythonPath` (a Python 3.10+ to run the servers instead of Houdini's) also apply to
APEX Script and OpenCL; put them under `"ko-apex"` or `"ko-opencl"` to set them for one language.
OpenCL also takes `includePaths` (extra header folders) and `defaultIncludes` (default `["matrix.h"]`)
under `"ko-opencl"`. Other VEX options: `mode`
(`"snippet"` or `"source"`), `context` (e.g. `"cvex"`), `compilerTimeout` (seconds),
`diagnosticsDebounceMs`, `diagnosticsMaxProblems`.

## Troubleshooting

- **Highlighting works but no completion or hover:** run **zed: open log** and look for `ko-vex`,
  `ko-apex` or `ko-opencl` errors. A `process:exec … not listed` error means the extension is outdated: run the
  package script again and rebuild it.
- **Installing the extension fails with "cannot find the `wasm32-wasip2` target":** Zed's default
  Rust is too old. Quit Zed completely, install the pinned toolchain
  (`rustup toolchain install 1.99.1 --profile minimal --target wasm32-wasip2`), start Zed with
  `rustup run 1.99.1 "$env:LOCALAPPDATA/Programs/Zed/bin/zed.exe"`, and install again.
- **Edit in Zed opens nothing:** check `KO_ZED_EXE`, and that the Houdini Zed window's title still
  shows its workspace name (`Houdini-Zed-01`).
- **VEX reports "validation unavailable":** the snippet uses an attribute of unknown type; add a
  prefix such as `v@` or set `attributeTypes`.
- **A VEX rename is refused:** the warning explains why. Only local variables and parameters can be
  renamed, and only when the compiler confirms every reference.

## For developers

- Offline tests (no Houdini, Zed or network): `python -m unittest discover -s tests/ko`. Set
  `KO_VEX_TEST_HFS` to a Houdini install folder to also run the tests that use its compiler,
  APEX function list and OpenCL headers.
- Extension tests: `cargo test --manifest-path editors/zed-houdini/Cargo.toml`.
- Edit-in-Zed shortcut GUI test: `./scripts/zed_gui_test.ps1`. Headless Houdini smoke test:
  `hython tests/ko/zed_houdini_smoke.py`.
- Layout: `editors/zed-houdini` (Zed extension), `editors/tree-sitter-vex` and
  `editors/tree-sitter-opencl` (grammars; `npm ci`, `npm run generate`, `npm test`), `tools/vex-lsp`,
  `tools/apex-lsp` and `tools/opencl-lsp` (language servers, bundled into the extension). The OpenCL
  built-in list is generated from clang's `opencl-c.h` by `scripts/opencl_builtins.py`; `editors/houdini-python` (Python stubs and settings), `python3.13libs/ko_zed*.py`
  (the Houdini side).

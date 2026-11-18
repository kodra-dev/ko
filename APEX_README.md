# APEX Script in Zed

Windows / Houdini 22. APEX Script support ships in the **Houdini Languages** Zed dev extension
(id `ko-houdini`, also providing VEX; `build/zed-houdini/zed-extension`, see [VEX_README.md](VEX_README.md)). Files ending in `.apex` open as
**APEX Script**; snippets from **Edit in Zed** on an APEX Script SOP already use that suffix.
No project setup, Houdini session, license, `hou` import, or scene cooking is involved.

## Setup

1. Regenerate the package: `./scripts/houdini_package.ps1`.
2. In Zed, uninstall the older **Houdini VEX** (id `ko-vex`) dev extension if present, then
   **Extensions → Install Dev Extension** and select `build/zed-houdini/zed-extension`.
   Restarting the language server alone does not load new languages.
3. Open any `.apex` file.

The `ko-apex` server uses the same Houdini/Python discovery as `ko-vex`. A `houdiniPath` or `pythonPath`
set under `lsp.ko-vex.settings` is inherited; `lsp.ko-apex.settings` overrides it for APEX only:

```json
{ "lsp": { "ko-apex": { "settings": { "houdiniPath": "C:/Program Files/Side Effects Software/Houdini 22.0.466" } } } }
```

## What you get

- **Highlighting, brackets, indentation and outline** on Zed's built-in Python grammar. The extension
  declares no Python grammar of its own, so ordinary Python files are unaffected.
- **Completion** for:
  - `apex.` namespaces and functions (`apex.sop.`, `apex.geo.`, …);
  - methods of typed values: `geo.`, `graph.`, `node.`, `port.`, vectors and arrays;
  - global functions, APEX types, constants (`M_PI`, `RED`, …), special functions (`BindInput`,
    `BindOutput`, `HoudiniVersion`, `InvokeOutputs`, …) and `@` decorators (`subgraph`, `namespace`,
    `safeguard_inputs`);
  - your own functions and variables, and `name=` keyword arguments (plus `__name`, `__color`,
    `valuetype`, …) inside calls.
- **Hover** with the APEX signature and SideFX's description, including in-place behaviour.
- **Signature help** while typing arguments, with the active positional or keyword parameter.
- **Go to definition** for local functions and variables. For APEX functions it opens Houdini's
  generated `apexscript.py` (read-only reference).
- **Document symbols** (functions and variables).
- **Diagnostics** while typing:
  - Python syntax errors, from Houdini's own Python parser (the one APEX Script is transpiled from);
  - constructs Houdini's transpiler rejects, with Houdini's own messages: `while`, `class`, `try`,
    `with`, `match`, `global`, `yield`, `break`, `continue`, `assert`, `del`;
  - imports, as a warning. The `from apexscript import *` line Houdini adds to saved snippet files is
    not reported.

Types come from literals, constructors (`Float(…)`), annotations (`x: Geometry = BindInput()`),
`BindInput(Geometry())`, function return types, and loops (`for pt in geo.points()` gives `Int`).
APEX Script is statically typed, so a variable keeps the type of its first assignment. Unknown types
simply give no member completion; they are never reported as errors.

## Where the information comes from

- Function catalogue: `$HFS/packages/apex/python3.*libs/apexscript.py`, which Houdini generates for
  its own snippet-editor autocomplete. It is parsed with `ast`, never imported or executed, and cached
  as JSON in the extension's work directory (one file, replaced when Houdini changes).
- Node and port methods (`a.name()`, `port.connect()`), special functions, decorators and constants
  come from the APEX Script documentation and Houdini's transpiler (`apex/graphscriptutils.py`,
  `apex/apexscript/builtins.py`), because the generated module omits them.
- First-letter case is ignored when matching function names, as Houdini does (`Sine()` is `sine`).
  Templated spellings such as `setPrimAttribValue_String` resolve to the base function.

## Limits

- **No undefined-name or unknown-function errors.** Header templates define names such as `graph`, and
  saved `@subgraph` functions become global functions, so these cannot be checked offline.
- No type checking, wrong-argument errors or graph errors; those still need Houdini. The APEX Script
  SOP's own errors (shown in Houdini) remain the reference.
- No rename or find references.
- APEX functions saved as subgraphs (`$HOUDINI_PATH/apexgraph`) and header/footer code are not
  indexed.
- **APEX AutoRig Component** snippets may still export from Houdini as `.py`, depending on the
  parameter's language tag. Rename them to `.apex`, or use Zed's `file_types` setting, until that is
  checked.
- Language behaviour inside Zed still needs the manual acceptance below.

## Tests

Offline (no Houdini needed):

```powershell
python -m unittest discover -s tests/ko -p 'test_apex*.py' -v
```

Against an installed Houdini's real catalogue (still no `hou`, scene, or license):

```powershell
$env:KO_VEX_TEST_HFS = 'C:/Program Files/Side Effects Software/Houdini 22.0.466'
python -m unittest discover -s tests/ko -p 'test_apex*.py' -v
cargo test --manifest-path editors/zed-houdini/Cargo.toml
Remove-Item Env:KO_VEX_TEST_HFS
```

The highlight, bracket, indent, outline and override queries were checked against tree-sitter-python
0.25.0, the version Zed is built with. Zed refuses to load a language whose queries name node types
that the grammar does not have.

Manual Zed acceptance:

1. Open a `.apex` file: it shows **APEX Script** in the status bar, with highlighting. Python files
   still use Python.
2. Type `geo = BindInput(Geometry())`, then `geo.`: Geometry methods are listed. Try `apex.sop.`.
3. Hover `addNode` in `graph.addNode(...)` (with `graph: ApexGraphHandle = BindInput()`).
4. Type `apex.sop.box(`: signature help shows parameters; `t=` is offered.
5. Add `while True:`: the error appears without saving and clears when it is removed.
6. Edit an APEX Script SOP snippet with **Edit in Zed** and confirm the same features.

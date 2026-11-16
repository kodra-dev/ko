# Edit Houdini code in Zed

Windows / Houdini 22. No provider, Zed extension, or blocking external-editor preference required.

## Use

1. Restart Houdini after installing/updating this package (it must be on `HOUDINI_PATH`).
2. Right-click a string/code **parameter label** → **Edit in Zed**, or focus the field and press **Alt+E**.
   Right-click a **shelf tool** → **Edit in Zed** edits that tool's script the same way; saving
   writes it back to the tool (and its `.shelf` file). Tool sessions survive loading another HIP.
3. Save in Zed. After roughly 350–700 ms of idle UI time, the saved text is copied back into that parameter.
   Houdini remains usable; do not close the Zed tab to apply changes.
4. **Ko → Zed → Zed Sessions…** lets you apply manually, reload one saved file from Houdini,
   toggle auto-apply for a binding, or detach it. Detaching retains the file and editor tab.
5. **Sync Houdini → Zed** on **KoDev** or under **Ko → Zed** refreshes **all attached editing files**
   from their Houdini parameters. It does not use node selection or attach new sessions.
   Changed saved files get adjacent `.backup-*` recovery copies before atomic replacement; successful
   exports clear conflict pauses and preserve each session's auto/manual setting. Failed exports stay
   paused and are reported without stopping the remaining exports. Unsaved Zed buffers are not replaced:
   resolve any reload/conflict prompt in Zed yourself.

A first request claims a persistent workspace slot, normally `Houdini-Zed-01`. Further requests open
tabs in that workspace, even if an unrelated Zed window has focus. Reopening the same parameter reuses
its file. Closing the Zed window does not release the slot; the next request reopens the same workspace.
The slot belongs to the Houdini bridge lifetime, not a particular snippet or HIP file.

The Zed CLI runs on a serialized background worker, never on Houdini's UI thread. No `--wait` is used.
Window enumeration checks the unique workspace name in the native window title before choosing
`--new` or `--existing`. A missing window is first opened using **only the persistent root folder**,
so Zed can restore the saved workspace layout. Once identified, that root plus the snippet are passed
to `--existing` to route the file. Startup requests are serialized and wait for window identification
**off the UI thread**.
An uncertain startup refuses further launches rather than repeatedly spawning possible duplicates.
The existing preview-tab setting is written only when a slot's `.zed/settings.json` is first created;
subsequent openings preserve any edits you made to that file.

## Remember folded docks without changing normal Zed preferences

1. Restart Houdini after this update, then use **Alt+E** or **Edit in Zed** to open the new persistent slot.
2. Fold the side docks in that **Houdini-Zed-01** window using Zed's dock toggle buttons.
3. Close that Zed workspace normally once, then open a snippet again. Zed restores its saved dock layout.

This is a **one-time layout setup per slot**, not forced first-launch dock folding. No global
`project_panel.starts_open` setting is changed, no keyboard/UI automation is sent to Zed, and no
Zed profile/authentication/database files are edited. Your unrelated projects retain their own layout.
A persistent workspace identity lets Zed's own workspace-state restoration do the work.

Concurrent Houdini bridges claim the lowest free slot (`Houdini-Zed-01`, `02`, etc., up to 32). Real
OS file locks prevent two live bridges from sharing a slot; locks are released on process exit/crash,
without deleting stale PID files. Each slot has its own saved layout, so fold docks once in `02` too
if you regularly run simultaneous Houdini sessions. Slot ownership lasts until shutdown/exit, and an
in-flight background CLI request retains ownership until it finishes. Shutdown cancels queued opens.

Old exported code is kept for recovery, but a new bridge starts with **no attached parameters**.
Reopening a parameter exports a fresh file; restored old Zed tabs never automatically reconnect to
a new scene. Old PID/random-named workspace folders from earlier versions are not migrated or deleted;
close their Zed windows yourself and use the new stable slot. Changing `KO_ZED_WORKSPACE` changes the
workspace identity and requires setting up its layout again.

Dock restoration is delegated to Zed, not forced on every snippet open: if you deliberately reopen a
dock in the Houdini window, that becomes its saved layout. Actual Zed dock restoration still needs
manual GUI acceptance on your installed Zed version.

## Focused-field shortcuts (experimental, automatically enabled)

Enabled automatically from `python3.13libs/uiready.py`, deferred until the UI event loop starts.
The existing keymap save runs **before** shortcut activation, so these runtime changes are not saved
into `Ko.keymap2`. Startup errors produce a warning instead of interrupting Houdini.
Set `KO_ZED_SHORTCUTS=0` before launching Houdini to opt out of automatic activation.

To activate the updated mapping without restarting an already-open Houdini, run in its Python Shell:

```python
import importlib, ko_zed_shortcuts
ko_zed_shortcuts.disable()
importlib.reload(ko_zed_shortcuts)
ko_zed_shortcuts.enable()
```

While editing a parameter, **Ctrl+E → internal editor**, **Alt+E → Zed**, for both single-line string
fields and multiline code fields. No unfocusing or hovering the label is required. Disable via the
**Ko → Zed → Toggle Zed Shortcuts**, `ko_zed_shortcuts.disable()`, or `ko_zed.shutdown()`.

This does **not** override a supported Open Editor callback: none was exposed in the native widgets
examined on Houdini 22.0.466 (no QAction; the parameter UI is drawn in native OpenGL widgets).
Instead, a Qt filter observes Alt+E in native `parmdialog` windows and lets the original key reach
Houdini's Open Editor command. It intercepts the resulting `Edit: /absolute/parameter/path` window
before display. No synthetic keystrokes or global input hooks are used. Houdini resolves the exact
field and commits pending input. Only then does
our handoff resolve that literal string parameter and call the existing Zed bridge. The transient
native editor is closed without editing its contents. This avoids guessing from node selection,
mouse position, or matching code text.

- Ctrl+E and Alt+E are added to the two native editor commands only when absent. Alt+E must reach
  the native command to resolve the field; only its resulting window is redirected. The same two
  chords on **our Edit in Zed parameter-menu action** are temporarily suspended because they compete
  with native field dispatch. Disabling removes additions and restores those suspended bindings.
  Other assignments are left alone. No keymap file is saved or edited. Don't reconfigure these
  assignments or save this temporary runtime layout as your keymap while the handoff is enabled.
- Keep **Preferences → Scripting → Use External Editor** off. That blocking external-editor route
  does not create the internal window required for target resolution.
- The filter applies to native parameter panels, not Python shells, network editors, or unrelated Qt fields.
- Existing _visible_ internal editor windows are not intercepted, to avoid discarding unsaved text.
  Close an internal editor before handing that same field to Zed. Unrecognized/non-string/expression
  editors remain native; a timeout reports that nothing was sent to Zed.
- This depends on Houdini 22's native widget/window names and English `Edit: /...` title. It is a
  tested workaround, not a stable public API. Revalidate it across Houdini upgrades; opt out with
  `KO_ZED_SHORTCUTS=0` if necessary.
- If attachment throws, the native editor is retained. A later asynchronous Zed CLI failure is
  reported by the bridge normally; the parameter and exported editing file remain intact.
- Real hidden-desktop GUI tests passed for focused single-line/multiline input, uncommitted text,
  held modifiers, repeated handoffs, both native Ctrl+E routes, unrelated Qt inputs, failed attachment,
  automatic uiready activation, keymap-save ordering, and disable cleanup. The test reproduces native Ctrl+E removed plus our menu action bound to Ctrl+E.
  Zed launch is mocked in these tests; the native Houdini editor and key handling are real.

## Safety and limitations

- **Auto-apply is initially on.** Saving Python code can execute it through normal node cooking;
  saving other code can trigger expensive cooks. Use Zed Sessions → Toggle Auto Apply or Houdini's manual
  update mode when needed. This tool does not force cooks when Houdini's update mode is manual.
- Each applied save is a Houdini undo group. Undoing then saving a different Zed revision is treated as
  a concurrent Houdini edit, not silently overwritten.
- If both parameter and editing file changed, automatic application pauses. Zed Sessions shows the
  conflict; **Apply File** explicitly overwrites the parameter, or **Reload Houdini** replaces the saved
  file after backing it up. Reload does not discard unsaved text inside Zed; resolve that editor prompt yourself.
- Literal strings only: keyframed/expression/reference parameters and locked parameters are rejected.
  Raw strings preserve `$variables`, backticks, Unicode and empty code. No `evalAsString()` is used.
- Binding targets do not follow selection. Deleted nodes detach on the next tick. Loading/clearing a HIP
  detaches all bindings so old tabs cannot update a replacement scene. Reattach explicitly in the new scene.
- Applied code is stored in the parameter/HIP; the scene does not depend on these files. Saved edits
  still need a normal HIP save. Unsaved Zed buffer changes are not synchronized.
- Keep this Zed window dedicated to Houdini, and keep its workspace folder name visible in the window
  title. Custom titles that hide it prevent identification; duplicated copies of the same workspace are rejected.
  Do not open this generated workspace in your unrelated windows. Zed has no public window-ID targeting;
  closing/moving its workspace during a launch remains a routing race. This is not an embedded editor API.
- Windows routing only. The routing decisions are covered offline, and the matching implementation was
  checked against installed Zed 1.28.1's source. An actual GUI routing acceptance run is still required.
- File suffixes: VEX `.vfl`, Python `.py`, APEX Script `.apex`, OpenCL `.cl`, other strings `.txt`.
  APEX does not deliberately masquerade as Python even though Houdini tags it as Python.
  Language highlighting/LSP/semantic rename support in Zed is separate from this bridge.
  Standalone VEX tooling and setup are now documented in [VEX_README.md](VEX_README.md).
- Text files are UTF-8, up to 4 MiB. Files remain under the workspace for recovery; no automatic deletion.
- The existing **Reload Ko** action preserves the live bridge. For a clean developer restart, run
  `import ko_zed; ko_zed.shutdown()` first. Shutdown leaves Zed and saved editing files intact.

## Configuration

Set before launching Houdini:

- `KO_ZED_EXE`: full path to the **CLI** executable, e.g.
  `C:/Users/you/AppData/Local/Programs/Zed/bin/zed.exe`. Not a shell command; no arguments.
  Otherwise PATH is searched, then the standard per-user Windows installation.
- `KO_ZED_WORKSPACE`: base directory for persistent workspace slots (prefer a local filesystem). Default:
  `$HOUDINI_USER_PREF_DIR/zed-editing`.

Optional: hide SideFX Labs' **Edit in External Editor [Labs]** entries (parameter and shelf tool
menus). Labs is ahead of this package on `HOUDINI_PATH` (see below), and menu files are parsed from the
back of the path forwards, so this package's own menu files are read before the Labs items exist. The
removals therefore live in `menu_overrides/`, which must be put first on the menu path. In the package
JSON that loads this repo:

```json
{ "var": "HOUDINI_MENU_PATH", "value": "$KO/menu_overrides;&" }
```

Package order: Houdini scans `$HOUDINI_USER_PREF_DIR/packages`, then `$HSITE`, `$HOUDINI_PACKAGE_DIR`,
and `$HFS/packages` last (which pulls in `sidefx_packages`, i.e. Labs), reading each directory's files
alphabetically. Every `hpath` is prepended, so the package read last ends up first.
If Labs is disabled, drop that line, or Houdini reports "Could not find element" at startup.

Existing `EDITOR`, `VISUAL`, Zed user settings, and authentication files are not modified or copied.

Direct Houdini Python UI-thread entry point:

```python
import ko_zed
ko_zed.edit_parameter(hou.node('/obj/geo1/attribwrangle1').parm('snippet'))
```

## Tests

Offline (does not launch Zed or Houdini):

```powershell
python -m unittest discover -s tests/ko -v
```

Native shortcut handoff GUI regression (isolated preferences, inactive desktop, no Zed/provider):

```powershell
./scripts/zed_gui_test.ps1
```

Real HOM smoke, headless, no Zed/provider:

```powershell
& 'C:/Program Files/Side Effects Software/Houdini 22.0.466/bin/hython.exe' tests/ko/zed_houdini_smoke.py
```

Manual GUI acceptance (not run automatically):

1. Leave an unrelated project open in Zed. Edit a wrangle through Houdini: a new window appears.
2. Focus the unrelated window, then edit a second wrangle: a tab appears in the Houdini window only.
3. Repeat the same wrangle: no duplicate tab or file. Rapidly request two snippets on cold startup: one window.
4. Close the Houdini Zed window; edit again: a fresh dedicated window, unrelated work unchanged.
5. Save VEX/Python/APEX/OpenCL code and verify the correct parameter updates without closing its tab.
6. Change code in both applications; verify conflict pause and both explicit resolution paths.
7. Rename/delete a node, undo an application, load/clear a HIP, and verify no accidental retargeting.
8. Toggle manual apply; verify saves do not update Houdini until Apply File.
9. Fold the side docks in slot `01`, close its Zed window normally, and reopen a snippet. Verify the
   folded layout returns and unrelated projects' docks stay unchanged. Repeat after restarting Houdini.
10. Run a second Houdini process: verify it uses slot `02`, with independent layout and parameter bindings.
    Exit/crash one process and verify a new bridge can reclaim that free slot without reconnecting old tabs.

References: https://zed.dev/docs/reference/cli and the installed Zed source at commit
`e8bc16acee5ae1937600d571d441c4b5d7b0d288` (`crates/zed/src/zed/open_listener.rs`,
`crates/project/src/project.rs`).

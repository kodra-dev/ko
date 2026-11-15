# Toggle Parameters

**Restart Houdini once** to discover the new menu item and remove the obsolete shelf tool.
Use **Ko → Toggle Parameters**, or **Ctrl+Shift+P** when that key is available.

- Open: a native floating Parameters pane without network controls or a tab bar.
- Open already: close it, saving its last normal size and position.
- Closed with X: reopen at the saved size and position, too.
- Normal geometry survives maximized/minimized closing, repeated toggles and restarts.
- Pin/node state is retained when the node still exists; reopening does not select it.
- A known existing single-tab floating `KoParameters`, `KoRestoreParameters`, or `FloatingPanel`
  is adopted, not duplicated. Docked Parameters panes and multi-tab floating workspaces are untouched.

The implementation uses the same **native HOM FloatingPanel position/size** approach as Toggle Loom:
creation receives the saved geometry, and native setters finalize it synchronously. No Qt
`saveGeometry`/frame-coordinate conversion, hardcoded screen position, delayed relocation or nested exec.
It does not depend on Loom, Checkpoints, or an AI provider.

Preferences are stored at `$HOUDINI_USER_PREF_DIR/ko-parameters.json` (via
`hou.homeHoudiniDirectory()`). `KO_PARAMETERS_STATE` can override the file for isolated testing.
The old shelf tool's Qt/QSettings geometry is deliberately not imported: it used a different,
unreliable coordinate system. An existing open window provides a fresh native measurement.
If no old window is open on first use, place/resize the new window once.

Startup only attaches tracking to an existing window; it never opens one automatically.
`uiready.py` configures the menu key before saving the Ko keymap. Existing custom menu assignments
and unrelated conflicting shortcuts are preserved. The old shelf action's Ctrl+Shift+P assignment
is migrated to the menu action; if another command occupies it, use the Hotkey Manager instead.

## Source

- `python3.13libs/ko_parameters.py`: native toggle, lifecycle tracking, atomic state and key migration.
- `MainMenuCommon.xml`: `h.ko_toggle_parameters`, under Ko.
- `python3.13libs/uiready.py`: startup adoption and shortcut setup.
- The generated `ko_restore_parameters` tool/member was removed from `toolbar/ko_network.shelf`.

Python entry points:

```python
import ko_parameters
ko_parameters.install()            # track/adopt only, never create a window
ko_parameters.toggle_parameters()  # close/open
```

Menu XML changes require a restart; Loom package reload does not reload this independent ko module.

## Offline tests

```powershell
python -m unittest discover -s tests/ko -p test_parameters.py
python -m unittest discover -s tests/ko -p test_shelves.py
python -m unittest discover -s tests/ko -p test_zed_shortcuts.py
./scripts/parameters_gui_test.ps1  # hidden Win32 desktop; Houdini 22.0.429 by default
./scripts/parameters_gui_test.ps1 -Houdini 'C:/Program Files/Side Effects Software/Houdini 22.0.466/bin/houdini.exe'
```

GUI tests use isolated preferences, read the Ko keymap without changing the source file, never switch
the input desktop or send physical/global input, and never invoke a provider. Coverage includes real
native menu scripts, actual Ctrl+Shift+P dispatch, adopting a normal floating Parameters pane,
selection/pinning, native close, fresh-controller persistence, three toggle cycles without drift,
maximized close, and module reload. Tests passed on both 22.0.429 and 22.0.466.

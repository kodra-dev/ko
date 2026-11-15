import os
import hou

# Native menu action: track existing Parameters windows without opening one.
# Migrate its persistent key BEFORE saving Ko, unlike temporary editor bindings.
if hou.isUIAvailable():
    try:
        import ko_parameters
        ko_parameters.install()
    except Exception as error:
        hou.ui.setStatusMessage("Toggle Parameters not initialized: " + str(error),
                                severity=hou.severityType.Warning)

# Save the user's keymap BEFORE installing temporary runtime editor bindings.
hou.hotkeys.saveAsKeymap("Ko", hou.text.expandString("$KO/Ko.keymap2"))


def _enable_zed_shortcuts():
    try:
        import ko_zed_shortcuts
        ko_zed_shortcuts.enable()
    except Exception as error:
        # Optional integration failure must not interrupt Houdini startup.
        hou.ui.setStatusMessage("Zed shortcuts not enabled: " + str(error),
                                severity=hou.severityType.Warning)


if hou.isUIAvailable() and os.environ.get("KO_ZED_SHORTCUTS", "1") != "0":
    from hutil.Qt import QtCore
    QtCore.QTimer.singleShot(0, _enable_zed_shortcuts)

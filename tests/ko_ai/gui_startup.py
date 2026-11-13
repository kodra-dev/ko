"""Real graphical regression test. Launched only by scripts/ko_ai_gui_test.ps1.

Uses a Qt timer (not a Houdini idle callback), so native pane operations are
actually processed by the desktop. Root existence alone is NOT a UI success:
check interface binding, Qt parenting, visibility, and controller liveness.
"""
import json
import os
from pathlib import Path
import sys
import time
import traceback

import hou
from PySide6 import QtCore, QtWidgets

DIRECTORY = Path(os.environ["KO_AI_GUI_TEST_DIR"])
REPO = Path(os.environ["KO_AI_GUI_REPO"])
sys.path.insert(0, str(REPO / "python3.13libs"))


def log(text):
    with (DIRECTORY / "gui.log").open("a", encoding="utf-8") as stream:
        stream.write(str(text) + "\n")


class GuiSmokeTest:
    def __init__(self):
        self.stage = 0
        self.pane = None
        self.duplicate = None
        self.next_run = time.monotonic() + 3
        self.timer = QtCore.QTimer(hou.qt.mainWindow())
        self.timer.timeout.connect(self.tick)
        self.timer.start(150)

    def check(self):
        interface = self.pane.activeInterface()
        assert interface is not None, "Native pane lost its active interface"
        assert interface.name() == "ko_ai_workspace"
        assert not self.pane.activeInterfaceScriptErrors(), self.pane.activeInterfaceScriptErrors()
        host = self.pane.activeInterfaceRootWidget()
        assert host is not None and host._content is not None, "Missing real controller"
        assert host.isVisible(), "Root exists but is not visible in Houdini"
        assert host.parent() is not None, "Root is not embedded in the native pane"
        assert not host._closed and not host._content._closed, "Pane has a closed controller"
        content = host._content
        if hasattr(self, "workspace_id"):
            assert content.workspace.history.id == self.workspace_id
            assert [r["id"] for r in content.workspace.history.entries] == self.checkpoint_ids
            assert content.workspace.active == self.checkpoint_ids[-1]
            assert hou.node("/obj/ko_ai_gui_test/box1").parm("sizex").eval() == 3
            assert any(m["text"] == "GUI_RELOAD_MARKER" for m in content.messages)
        log(f"PASS stage {self.stage}: visible, embedded, live, bound pane")
        return host

    def finish(self, error=None):
        self.timer.stop()
        result = {"passed": error is None, "houdini": hou.applicationVersionString(),
                  "last_stage": self.stage, "error": error}
        (DIRECTORY / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        log(result)
        hou.exit(exit_code=1 if error else 0, suppress_save_prompt=True)

    def tick(self):
        if time.monotonic() < self.next_run:
            return
        self.next_run = time.monotonic() + 1
        try:
            import ko_ai
            log(f"STAGE {self.stage}")
            if self.stage == 0:
                self.pane = ko_ai.open_pane()
            elif self.stage == 1:
                host = self.check()
                geo = hou.node("/obj").createNode("geo", "ko_ai_gui_test", run_init_scripts=False)
                box = geo.createNode("box", "box1")
                a = host._content.workspace.checkpoint("Original")
                box.parm("sizex").set(3)
                b = host._content.workspace.checkpoint("Wide")
                host._content._append("You", "GUI_RELOAD_MARKER")
                self.workspace_id = host._content.workspace.history.id
                self.checkpoint_ids = [a["id"], b["id"]]
                host._content.refresh_versions()
                self.old_host = host
                reload_button = next(w for w in host.findChildren(QtWidgets.QPushButton)
                                     if w.text() == "Reload tool")
                reload_button.click()
            elif self.stage == 2:
                host = self.check()
                assert host is not self.old_host, "Reload button did not replace the interface"
                ko_ai.reload()
            elif self.stage == 3:
                self.check()
                self.pane.reloadActiveInterface()
            elif self.stage == 4:
                self.check()
                self.duplicate = hou.ui.curDesktop().createFloatingPaneTab(
                    hou.paneTabType.PythonPanel, python_panel_interface="ko_ai_workspace", immediate=True)
            elif self.stage == 5:
                self.check()
                notice = self.duplicate.activeInterfaceRootWidget()
                assert notice is not None and notice.isVisible()
                assert notice._content is None and "two panes" in notice.message.text()
                self.duplicate.close()
                # Reproduce the previous bug deliberately; then test recovery.
                hou.pypanel.installFile(str(REPO / "python_panels/ko_ai.pypanel"))
                log("After legacy reinstall, interface = " + str(self.pane.activeInterface()))
            elif self.stage == 6:
                ko_ai.reload()
            elif self.stage == 7:
                host = self.check()
                host.window().grab().save(str(DIRECTORY / "reloaded-pane.png"))
                self.finish()
                return
            self.stage += 1
        except Exception:
            self.finish(traceback.format_exc())


log("Started isolated GUI test: " + hou.applicationVersionString())
test = GuiSmokeTest()

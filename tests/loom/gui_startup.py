"""Real graphical regression test. Launched only by scripts/loom_gui_test.ps1.

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
from PySide6 import QtCore, QtGui, QtTest, QtWidgets

DIRECTORY = Path(os.environ["LOOM_GUI_TEST_DIR"])
REPO = Path(os.environ["LOOM_GUI_REPO"])
sys.path.insert(0, str(REPO / "python3.13libs"))


def log(text):
    with (DIRECTORY / "gui.log").open("a", encoding="utf-8") as stream:
        stream.write(str(text) + "\n")


def current_desktop():
    """Observe the GUI thread's assigned desktop; never activate or switch it."""
    import ctypes
    from ctypes import wintypes
    user32 = ctypes.WinDLL("user32", use_last_error=True)
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.GetCurrentThreadId.restype = wintypes.DWORD
    user32.GetThreadDesktop.argtypes = [wintypes.DWORD]
    user32.GetThreadDesktop.restype = wintypes.HANDLE
    user32.GetUserObjectInformationW.argtypes = [wintypes.HANDLE, ctypes.c_int,
                                                wintypes.LPVOID, wintypes.DWORD,
                                                ctypes.POINTER(wintypes.DWORD)]
    user32.GetUserObjectInformationW.restype = wintypes.BOOL
    handle = user32.GetThreadDesktop(kernel32.GetCurrentThreadId())
    buffer = ctypes.create_unicode_buffer(256)
    needed = wintypes.DWORD()
    if not user32.GetUserObjectInformationW(handle, 2, buffer, ctypes.sizeof(buffer), ctypes.byref(needed)):
        raise ctypes.WinError(ctypes.get_last_error())
    return buffer.value


def loom_callbacks():
    return [callback for callback in hou.hipFile.eventCallbacks()
            if getattr(callback, "__module__", "").startswith("loom.")]


def trigger_menu(symbol, label):
    """Invoke the XML menu command through Houdini's native hotkey dispatcher.

    Houdini's main menubar is native-drawn, not a QMenuBar. Bind a test-only
    shortcut in our isolated preferences, then send Qt events only to this
    process on its hidden desktop; never send physical/global keyboard input.
    """
    assert hou.hotkeys.hotkeyLabel(symbol) == label
    if symbol != "h.loom_reload_package":
        # Houdini does not dispatch these newly rebound menu shortcuts reliably.
        # Execute the installed XML item\'s actual script, not a separate test callback.
        import xml.etree.ElementTree as ET
        source = ET.parse(REPO / "MainMenuCommon.xml").find(".//scriptItem[@id=\'" + symbol + "\']/scriptCode").text
        exec(compile(source, "MainMenuCommon.xml:" + symbol, "exec"), {})
        return
    # Reuse a known unshadowed global shortcut, clearing only our isolated test bindings.
    for command in ("h.loom_reload_package", "h.loom_open_agent", "h.loom_open_checkpoints"):
        hou.hotkeys.clearAssignments("h", command)
    assert hou.hotkeys.addAssignment("h", symbol, "Ctrl+Alt+Shift+F11")
    window = hou.qt.mainWindow()
    window.raise_()
    window.activateWindow()
    QtTest.QTest.qWait(60)  # Native cook scopes/window activation may finish asynchronously.
    QtTest.QTest.keyClick(window, QtCore.Qt.Key.Key_F11,
        QtCore.Qt.KeyboardModifier.ControlModifier | QtCore.Qt.KeyboardModifier.AltModifier
        | QtCore.Qt.KeyboardModifier.ShiftModifier)


def trigger_reload_menu():
    trigger_menu("h.loom_reload_package", "Reload Loom + Checkpoints")


class GuiSmokeTest:
    def __init__(self):
        self.stage = -1
        self.pane = None
        self.duplicate = None
        self.created = time.monotonic()
        self.next_run = self.created + 3
        self.timer = QtCore.QTimer(hou.qt.mainWindow())
        self.timer.timeout.connect(self.tick)
        self.timer.start(150)

    def check(self):
        assert current_desktop() == self.desktop, "GUI thread changed desktops"
        interface = self.pane.activeInterface()
        assert interface is not None, "Native pane lost its active interface"
        assert interface.name() == "loom_workspace"
        assert not self.pane.activeInterfaceScriptErrors(), self.pane.activeInterfaceScriptErrors()
        host = self.pane.activeInterfaceRootWidget()
        assert host is not None and host._content is not None, "Missing real controller"
        assert host.isVisible(), "Root exists but is not visible in Houdini"
        assert host.parent() is not None, "Root is not embedded in the native pane"
        assert not host._closed and not host._content._closed, "Pane has a closed controller"
        content = host._content
        if hasattr(self, "checkpoint_widget"):
            assert content.workspace is self.checkpoint_widget.workspace
            assert not self.checkpoint_widget._closed
            assert not self.checkpoint_pane.activeInterfaceScriptErrors()
        if hasattr(self, "workspace_id"):
            assert content.workspace.history.id == self.workspace_id
            assert [r["id"] for r in content.workspace.history.entries] == self.checkpoint_ids
            assert content.workspace.active == self.checkpoint_ids[-1]
            assert hou.node("/obj/loom_gui_test/box1").parm("sizex").eval() == getattr(self, "expected_size", 3)
            assert any(m["text"] == "GUI_RELOAD_MARKER" for m in content.messages)
        log(f"PASS stage {self.stage}: visible, embedded, live, bound pane")
        return host

    def finish(self, error=None):
        self.timer.stop()
        result = {"passed": error is None, "houdini": hou.applicationVersionString(),
                  "last_stage": self.stage, "error": error,
                  "desktop": getattr(self, "desktop", None),
                  "expected_desktop": os.environ.get("LOOM_GUI_TEST_DESKTOP")}
        (DIRECTORY / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        log(result)
        if self.pane is not None:
            root = self.pane.activeInterfaceRootWidget()
            if root is not None and hasattr(root, "shutdown"):
                root.shutdown()
                log("Agent shutdown complete")
        if getattr(self, "checkpoint_widget", None) is not None:
            self.checkpoint_widget.shutdown()
            log("Checkpoint shutdown complete")
        # Exit through Houdini's callback boundary, not a PySide timer callback:
        # hou.exit raises SystemExit, which Qt may swallow before queued teardown.
        def quit_when_idle():
            hou.ui.removeEventLoopCallback(quit_when_idle)
            hou.exit(exit_code=1 if error else 0, suppress_save_prompt=True)
        hou.ui.addEventLoopCallback(quit_when_idle)

    def tick(self):
        if time.monotonic() < self.next_run:
            return
        self.next_run = time.monotonic() + 1
        try:
            import loom
            log(f"STAGE {self.stage}")
            if self.stage == -1 and getattr(self, "checkpoint_pane", None) is None:
                # Houdini can still be (re)building its main window when startup scripts'
                # timers fire; panes floated then may be hidden. Require a stable window.
                main = hou.qt.mainWindow()
                if main is None or not main.isVisible():
                    self.main_stable_since = None
                    if time.monotonic() - getattr(self, "created", time.monotonic()) < 60:
                        self.next_run = time.monotonic() + 0.3
                        return
                    raise RuntimeError("Houdini main window never became visible")
                if getattr(self, "main_stable_since", None) is None:
                    self.main_stable_since = time.monotonic()
                if time.monotonic() - self.main_stable_since < 2:
                    self.next_run = time.monotonic() + 0.3
                    return
                log(f"Main window stable after {time.monotonic() - self.created:.1f}s")
            if self.stage == -1:
                self.desktop = current_desktop()
                expected = os.environ.get("LOOM_GUI_TEST_DESKTOP")
                log("GUI desktop: " + self.desktop)
                if expected:
                    assert self.desktop == expected, "Hidden launch used the wrong desktop"
                    assert self.desktop.lower() != "default", "Hidden launch used the visible Default desktop"
                # Checkpoints must open before the agent and remain alive across reloads.
                from loom import checkpoints
                if getattr(self, "checkpoint_pane", None) is None:
                    self.checkpoint_pane = checkpoints.open_pane()
                    self.checkpoint_started = time.monotonic()
                self.checkpoint_widget = self.checkpoint_pane.activeInterfaceRootWidget()
                if self.checkpoint_widget is None:
                    if time.monotonic() - self.checkpoint_started < 15:
                        return  # Creation can cross a real native idle boundary under load.
                    raise RuntimeError("Checkpoint root did not initialize: " + repr(
                        self.checkpoint_pane.activeInterfaceScriptErrors()))
                assert hasattr(self.checkpoint_widget, "workspace"), repr([
                    w.toPlainText() for w in self.checkpoint_widget.findChildren(QtWidgets.QPlainTextEdit)])
                # On a hidden desktop Houdini may still be showing its windows when this
                # timer starts; floating panes created then stay hidden for a while. Wait
                # (never force-show) until the first native floating pane is really shown.
                main = hou.qt.mainWindow()  # Can transiently be None during startup.
                if not (main is not None and main.isVisible() and self.checkpoint_widget.isVisible()):
                    if time.monotonic() - self.checkpoint_started < 60:
                        return
                    raise RuntimeError("Checkpoint pane never became visible: main_window="
                                       + repr(main is not None and main.isVisible()))
                log(f"Checkpoint pane visible after {time.monotonic() - self.checkpoint_started:.1f}s")
                # A saved legacy desktop still loads, then migrates to the new interface.
                hou.pypanel.installFile(str(REPO / "python_panels/loom_legacy.pypanel"))
                legacy = hou.ui.curDesktop().createFloatingPaneTab(
                    hou.paneTabType.PythonPanel, python_panel_interface="ko_ai_workspace", immediate=True)
                self.pane = legacy
                self.legacy_name = legacy.name()
                self.legacy_started = time.monotonic()
            elif self.stage == 0:
                # Let Houdini finish showing the legacy desktop pane before rebinding.
                root = self.pane.activeInterfaceRootWidget()
                if not root.isVisible():
                    if time.monotonic() - self.legacy_started < 8:
                        return  # Wait for the native floating window to finish showing; never force-show it.
                    chain, widget = [], root
                    while widget is not None:
                        chain.append((widget.objectName(), type(widget).__name__, widget.isVisible(), widget.isHidden(),
                                      widget.width(), widget.height()))
                        widget = widget.parentWidget()
                    floating = self.pane.floatingPanel()
                    log("Legacy visibility: " + repr(chain) + " errors=" + self.pane.activeInterfaceScriptErrors()
                        + f" elapsed={time.monotonic() - self.legacy_started:.1f}"
                        + f" floating={floating is not None}"
                        + f" checkpoint_visible={self.checkpoint_widget.isVisible()}"
                        + f" main_window={hou.qt.mainWindow() is not None and hou.qt.mainWindow().isVisible()}"
                        + " panes=" + repr([(p.name(), p.type().name(), p.floatingPanel() is not None)
                                            for p in hou.ui.paneTabs()]))
                assert root.isVisible()
                self.pane = loom.open_pane()
                assert self.pane.name() == self.legacy_name
                assert self.pane.activeInterface().name() == "loom_workspace"
            elif self.stage == 1:
                host = self.check()
                panel = host._content
                # Click from another native pane, not an already-focused editor.
                QtTest.QTest.mouseClick(self.checkpoint_widget.versions.viewport(), QtCore.Qt.MouseButton.LeftButton)
                QtTest.QTest.mouseClick(panel.prompt.viewport(), QtCore.Qt.MouseButton.LeftButton,
                                       pos=QtCore.QPoint(20, 20))
                QtTest.QTest.qWait(40)
                assert panel.prompt.hasFocus(), "One click did not focus the composer: " + repr((
                    QtWidgets.QApplication.focusWidget(), QtWidgets.QApplication.activeWindow(), panel.prompt.window()))
                QtTest.QTest.keyClicks(QtWidgets.QApplication.focusWidget(), "One click")
                QtTest.QTest.keyClick(panel.prompt, QtCore.Qt.Key.Key_Return,
                                     QtCore.Qt.KeyboardModifier.ShiftModifier)
                assert panel.prompt.toPlainText() == "One click\n"
                submitted = []
                panel.prompt.submitted.connect(lambda: submitted.append(True))
                QtTest.QTest.keyClick(panel.prompt, QtCore.Qt.Key.Key_Return)
                assert submitted == [True]
                assert panel.prompt.toPlainText() == "One click\n", "Enter inserted a newline instead of submitting"
                panel.prompt.clear()
                from loom.local import hotkeys
                assert hou.hotkeys.loadKeymap("Ko", str(REPO / "Ko.keymap2"))
                bindings = hotkeys("PolyExtrude")
                log("Live PolyExtrude bindings: " + json.dumps(bindings))
                assert any("Alt+E" in match["keys"] and "tool:sop_polyextrude" in match["symbol"]
                           for match in bindings["matches"]), repr(bindings)
                symbol = "h.pane.gview.tool:sop_polyextrude::2.0"
                assert hou.hotkeys.addAssignment("h.pane.gview", symbol, "Ctrl+Alt+E")
                updated = hotkeys("PolyExtrude")
                actual = list(hou.hotkeys.assignments("h.pane.gview", symbol))
                assert any(set(key.split("+")) == {"Ctrl", "Alt", "E"} for key in actual), repr(actual)
                assert any(match["symbol"] == symbol and match["keys"] == actual
                           for match in updated["matches"]), "Hotkey cache is stale: " + repr((actual, updated))
                assert hou.hotkeys.removeAssignment("h.pane.gview", symbol, "Ctrl+Alt+E")
                panel._append("You", "Do I have a shortcut to PolyExtrude now?")
                panel._append("Tool", json.dumps(bindings, indent=2),
                              summary="Inspected local hotkeys · PolyExtrude", success=True)
                panel._append("Loom", "Yes — **Alt+E** invokes **PolyExtrude** in the geometry viewer.\n\n"
                              "This is your live **Ko** keymap, not Houdini's stock shortcut list.\n\n"
                              "For example, a node path appears as `/obj/cube/polyextrude1`.")
                panel.status.setText("Offline UI preview · mock messages, no provider connected")
                QtTest.QTest.qWait(50)  # Allow inserted cards/layouts to become visible before capture.
                assert len(panel.transcript.cards) == 3
                assert all(card.isVisible() for card in panel.transcript.cards)
                assert "Alt+E" in panel.transcript.cards[-1].body.toPlainText()
                host.grab().save(str(DIRECTORY / "loom-chat.png"))
                geo = hou.node("/obj").createNode("geo", "loom_gui_test", run_init_scripts=False)
                box = geo.createNode("box", "box1")
                # Exercise actual HOM inspection + actual image payloads, with no provider.
                import base64
                import queue
                from unittest.mock import patch
                class CaptureTransport:
                    def __init__(self):
                        self.events = queue.Queue()
                        self.replies = []
                    def reply(self, request_id, result):
                        self.replies.append(result)
                    def close(self):
                        pass
                fake = CaptureTransport()
                panel.transport = fake
                panel._turn_workspace = panel.workspace.history.id
                panel._set_busy(True)
                try:
                    angle = panel.spinner.angle
                    QtTest.QTest.qWait(160)
                    assert panel.spinner.angle != angle
                    with patch.object(QtWidgets.QMessageBox, "question") as approval:
                        panel._event({"id": 900, "method": "item/tool/call", "params": {
                            "tool": "loom_scene", "arguments": {"operation": "geometry", "path": box.path()}}})
                        assert fake.replies[-1]["success"], repr(fake.replies[-1])
                        approval.assert_not_called()
                    panel._event({"id": 901, "method": "item/tool/call", "params": {
                        "tool": "loom_scene", "arguments": {"operation": "viewport"}}})
                    response = fake.replies[-1]
                    assert response["success"], repr(response)
                    assert response["contentItems"][1]["type"] == "inputImage"
                    data = base64.b64decode(response["contentItems"][1]["imageUrl"].split(",", 1)[1])
                    (DIRECTORY / "agent-viewport.png").write_bytes(data)
                    assert not any("base64" in message["text"] for message in panel.messages)
                    host.grab().save(str(DIRECTORY / "spinner-working.png"))
                    # Develop a real persistent module, then invoke it in live Houdini.
                    # All requests are mocked; the panes live only on the hidden test desktop.
                    panel._turn_allow_edits = True
                    panel._turn_permission_mode = "tool_dev"
                    feature_path = DIRECTORY / "floating_feature.py"
                    source = ("import hou\n\ndef open_parameters(node):\n"
                              "    pane = hou.ui.curDesktop().createFloatingPaneTab(hou.paneTabType.Parm)\n"
                              "    pane.setCurrentNode(node)\n    pane.setPin(True)\n    return pane\n")
                    panel._event({"id": 902, "method": "item/tool/call", "params": {
                        "tool": "loom_files", "arguments": {"requests_json": json.dumps([
                            {"op": "write", "path": str(feature_path), "content": source, "expected_sha256": ""}])}}})
                    assert fake.replies[-1]["success"], repr(fake.replies[-1])
                    assert feature_path.read_text() == source
                    code = ("import importlib.util\nfrom PySide6 import QtCore, QtWidgets\n"
                            "assert QtCore.QThread.currentThread() == QtWidgets.QApplication.instance().thread()\n"
                            f"spec = importlib.util.spec_from_file_location('floating_feature', {str(feature_path)!r})\n"
                            "feature = importlib.util.module_from_spec(spec)\nspec.loader.exec_module(feature)\n"
                            f"hou.session._loom_test_parms = feature.open_parameters(hou.node({box.path()!r}))\n"
                            "print(hou.session._loom_test_parms.currentNode().path())")
                    panel._event({"id": 903, "method": "item/tool/call", "params": {
                        "tool": "loom_python", "arguments": {"label": "Floating parameters", "code": code}}})
                    assert fake.replies[-1]["success"], repr(fake.replies[-1])
                    parm_pane = hou.session._loom_test_parms
                    assert parm_pane.type() == hou.paneTabType.Parm
                    assert parm_pane.currentNode() == box
                    assert parm_pane.isPin()
                    parm_pane.close()
                    del hou.session._loom_test_parms
                    log("Tool Dev wrote a module and executed it on the GUI thread to open pinned floating parameters")
                finally:
                    panel.transport = None
                    panel._set_busy(False)
                a = host._content.workspace.checkpoint("Original")
                box.parm("sizex").set(3)
                b = host._content.workspace.checkpoint("Wide")
                host._content._append("You", "GUI_RELOAD_MARKER")
                self.workspace_id = host._content.workspace.history.id
                self.checkpoint_ids = [record["id"] for record in panel.workspace.history.entries]
                host._content.refresh_all()
                self.old_host = host
                self.old_checkpoint = self.checkpoint_widget
                self.old_workspace_type = type(host._content.workspace)
                assert not any(w.text() == "Reload tool" for w in host.findChildren(QtWidgets.QPushButton))
                trigger_reload_menu()
            elif self.stage == 2:
                self.checkpoint_widget = self.checkpoint_pane.activeInterfaceRootWidget()
                host = self.check()
                assert host is not self.old_host, "Package menu did not rebuild the agent"
                assert self.checkpoint_widget is not self.old_checkpoint, "Package menu did not rebuild Checkpoints"
                assert type(host._content.workspace) is not self.old_workspace_type, "Checkpoint core was not reloaded"
                assert len(loom_callbacks()) == 1, "Reload leaked HIP callbacks: " + repr(loom_callbacks())
                hou.qt.mainWindow().grab().save(str(DIRECTORY / "loom-menu.png"))
                loom._reload_agent_pane()
            elif self.stage == 3:
                self.check()
                self.pane.reloadActiveInterface()
            elif self.stage == 4:
                self.check()
                self.duplicate = hou.ui.curDesktop().createFloatingPaneTab(
                    hou.paneTabType.PythonPanel, python_panel_interface="loom_workspace", immediate=True)
            elif self.stage == 5:
                self.check()
                notice = self.duplicate.activeInterfaceRootWidget()
                assert notice is not None and notice.isVisible()
                assert notice._content is None and "two panes" in notice.message.text()
                self.duplicate.close()
                # Reproduce the previous bug deliberately; then test recovery.
                hou.pypanel.installFile(str(REPO / "python_panels/loom.pypanel"))
                log("After legacy reinstall, interface = " + str(self.pane.activeInterface()))
            elif self.stage == 6:
                trigger_reload_menu()
            elif self.stage == 7:
                self.checkpoint_widget = self.checkpoint_pane.activeInterfaceRootWidget()
                host = self.check()
                checkpoint = self.checkpoint_widget
                checkpoint.refresh_versions()
                item = checkpoint.versions.topLevelItem(0)
                item.setSelected(True)
                key = item.data(0, QtCore.Qt.ItemDataRole.UserRole)
                self.checkpoint_pane.setIsCurrentTab()
                checkpoint.window().raise_()
                checkpoint.window().activateWindow()
                checkpoint.versions.setFocus()
                def rename():
                    from loom.checkpoints.ui import TextInputDialog
                    dialog = QtWidgets.QApplication.activeModalWidget()
                    if isinstance(dialog, TextInputDialog):
                        dialog.input.setText("GUI_F2_RENAMED")
                        QtTest.QTest.keyClick(dialog.input, QtCore.Qt.Key.Key_Return)
                QtCore.QTimer.singleShot(100, rename)
                QtTest.QTest.keyClick(checkpoint.versions, QtCore.Qt.Key.Key_F2)
                assert checkpoint.workspace.history.get(key)["label"] == "GUI_F2_RENAMED", "F2 did not rename in the native checkpoint pane"
                checkpoint.window().grab().save(str(DIRECTORY / "checkpoints-pane.png"))
                host.window().grab().save(str(DIRECTORY / "reloaded-pane.png"))
                # Conversation manager is a separate modeless window, not a dropdown.
                agent = host._content
                original = agent.conversation_id
                agent.new_chat()
                created = agent.conversation_id
                agent._append("You", "Model a beveled mechanical housing\nKeep the topology clean.",
                              turn_id="gui-recorded-turn", branch_point="before")
                manager = agent.manage_conversation()
                QtWidgets.QApplication.processEvents()
                manager.items.setFocus()
                def rename_conversation():
                    dialog = QtWidgets.QApplication.activeModalWidget()
                    if dialog is not None:
                        dialog.input.setText("Mechanical Housing")
                        QtTest.QTest.keyClick(dialog.input, QtCore.Qt.Key.Key_Return)
                QtCore.QTimer.singleShot(100, rename_conversation)
                QtTest.QTest.keyClick(manager.items, QtCore.Qt.Key.Key_F2)
                assert agent.conversations.current["name"] == "Mechanical Housing", "Manager F2 failed"
                manager.grab().save(str(DIRECTORY / "conversations-manager.png"))
                # Exercise the native message menu with a fake callback, never a provider.
                branch_requests, menu_errors = [], []
                branch_handler = agent.branch_conversation
                agent.branch_conversation = branch_requests.append
                card = agent.transcript.cards[0]
                def choose_branch():
                    menu = QtWidgets.QApplication.activePopupWidget()
                    try:
                        menu.grab().save(str(DIRECTORY / "conversation-branch-menu.png"))
                        action = next(a for a in menu.actions() if a.text() == "Branch here")
                        assert action.isEnabled(), "Recorded prompt branch action is disabled"
                        action.trigger()
                    except Exception as exc:
                        menu_errors.append(str(exc))
                    finally:
                        if menu is not None:
                            menu.close()
                QtCore.QTimer.singleShot(100, choose_branch)
                card._context_menu(QtCore.QPoint(2, 2))
                agent.branch_conversation = branch_handler
                assert not menu_errors, menu_errors
                assert len(branch_requests) == 1
                assert branch_requests[0]["_conversation_id"] == created
                assert branch_requests[0]["_branch_index"] == 0
                def choose_edit():
                    menu = QtWidgets.QApplication.activePopupWidget()
                    try:
                        next(a for a in menu.actions() if a.text() == "Edit prompt").trigger()
                    except Exception as exc:
                        menu_errors.append(str(exc))
                    finally:
                        if menu is not None:
                            menu.close()
                QtCore.QTimer.singleShot(100, choose_edit)
                card._context_menu(QtCore.QPoint(2, 2))
                assert not menu_errors, menu_errors
                assert agent.conversation_id == created and not agent.messages
                assert agent.prompt.toPlainText().startswith("Model a beveled mechanical housing")
                agent._append("You", "Question with tool activity")
                for n in range(3):
                    agent._append("Tool", "Fake action " + str(n))
                agent._append("Loom", "Yes, here is the reply")
                agent.trim_conversation(agent.transcript.cards[-1]._message)
                actions = agent.transcript.cards[1]
                assert actions.title.text() == "3 actions" and actions.body.isHidden()
                def delete_actions():
                    menu = QtWidgets.QApplication.activePopupWidget()
                    try:
                        menu.grab().save(str(DIRECTORY / "action-group-menu.png"))
                        next(a for a in menu.actions() if a.text() == "Delete from here").trigger()
                    except Exception as exc:
                        menu_errors.append(str(exc))
                    finally:
                        if menu is not None:
                            menu.close()
                QtCore.QTimer.singleShot(100, delete_actions)
                actions.title.customContextMenuRequested.emit(QtCore.QPoint(2, 2))
                assert not menu_errors, menu_errors
                assert [m["role"] for m in agent.messages] == ["You"]
                assert agent.conversation_id == created
                def conversation_item(key):
                    return next(manager.items.topLevelItem(i) for i in range(manager.items.topLevelItemCount())
                                if manager.items.topLevelItem(i).data(0, QtCore.Qt.ItemDataRole.UserRole) == key)
                item = conversation_item(original)
                rect = manager.items.visualItemRect(item)
                QtTest.QTest.mouseClick(manager.items.viewport(), QtCore.Qt.MouseButton.LeftButton, pos=rect.center())
                assert agent.conversation_id == created, "Selecting a row activated it"
                QtTest.QTest.mouseDClick(manager.items.viewport(), QtCore.Qt.MouseButton.LeftButton, pos=rect.center())
                assert agent.conversation_id == original, "Manager double-click did not activate"
                manager.items.setCurrentItem(conversation_item(created))
                manager.delete_button.click()
                assert agent.conversation_id == original
                assert all(r["id"] != created for r in agent.conversations.records)
                manager.close()
            elif self.stage == 8:
                host = self.check()
                box = hou.node("/obj/loom_gui_test/box1")
                box.parm("sizex").set(4)
                self.expected_size = 4
                # The selected row is Original, but capture must target current Wide.
                self.checkpoint_widget.capture_button.click()
                record = host._content.workspace.history.get(self.checkpoint_ids[-1])
                assert record.get("preview"), self.checkpoint_widget.status.text()
                image_path = host._content.workspace.history.directory(record["id"]) / record["preview"]
                image = QtGui.QImage(str(image_path))
                assert not image.isNull(), "Viewport capture is missing or unreadable"
                colors = {image.pixel(x, y)
                          for x in range(0, image.width(), max(1, image.width() // 32))
                          for y in range(0, image.height(), max(1, image.height() // 32))}
                assert len(colors) > 8, "Viewport capture is blank/unrendered on the test desktop"
                log(f"Viewport rendered: {image.width()}x{image.height()}, {len(colors)} sampled colors")
                self.checkpoint_widget.window().grab().save(str(DIRECTORY / "current-highlight.png"))
                self.checkpoint_widget.versions.clearSelection()
                self.checkpoint_widget.versions.topLevelItem(0).child(0).setSelected(True)
                self.checkpoint_widget.window().grab().save(str(DIRECTORY / "selected-current.png"))
                box.parm("sizex").set(5)
                self.expected_size = 5
                hou.hipFile.save(str(DIRECTORY / "renamed.hip"))
                self.working_path = hou.hipFile.path()
            elif self.stage == 9:
                host = self.check()
                workspace = host._content.workspace
                assert not workspace.save_warning, workspace.save_warning
                assert workspace.graph(self.checkpoint_ids[-1])["nodes"]["/obj/loom_gui_test/box1"]["parms"]["sizex"] == "5"
                assert not hou.hipFile.hasUnsavedChanges(), "Checkpoint sync dirtied the normally saved HIP"
                workspace.visit(self.checkpoint_ids[0])
                assert hou.hipFile.path() == self.working_path, "Visit undid Save As"
                workspace.visit(self.checkpoint_ids[-1])
                assert hou.hipFile.path() == self.working_path
                self.checkpoint_pane.reloadActiveInterface()
                self.checkpoint_widget = self.checkpoint_pane.activeInterfaceRootWidget()
            elif self.stage == 10:
                host = self.check()
                self.busy_host = host
                self.busy_checkpoint = self.checkpoint_widget
                host._content._set_busy(True)
                trigger_reload_menu()
            elif self.stage == 11:
                assert self.pane.activeInterfaceRootWidget() is self.busy_host, "Busy reload replaced agent"
                assert self.checkpoint_pane.activeInterfaceRootWidget() is self.busy_checkpoint, "Busy reload replaced Checkpoints"
                self.busy_host._content._set_busy(False)
                self.pane.close()
                self.pane = None
                trigger_reload_menu()
            elif self.stage == 12:
                self.checkpoint_widget = self.checkpoint_pane.activeInterfaceRootWidget()
                assert not self.checkpoint_pane.activeInterfaceScriptErrors()
                assert self.checkpoint_widget.workspace.active == self.checkpoint_ids[-1]
                assert len(self.checkpoint_widget.session.clients) == 1, "Checkpoints-only reload opened an agent"
                assert len(loom_callbacks()) == 1
                self.checkpoint_widget.make_checkpoint()
                assert self.checkpoint_widget.selected_ids() == [self.checkpoint_widget.workspace.active]
                self.checkpoint_widget.window().grab().save(str(DIRECTORY / "new-selected.png"))
                record = self.checkpoint_widget.workspace.checkpoint("Extrude top faces", created_by="ai")
                self.checkpoint_widget.refresh_versions(select_id=record["id"])
                item = self.checkpoint_widget.versions.selectedItems()[0]
                assert not item.icon(0).isNull()
                assert item.text(0) == "Extrude top faces"
                self.checkpoint_widget.versions.header().resizeSection(0, 900)
                self.checkpoint_widget.refresh_versions()
                assert self.checkpoint_widget.versions.columnWidth(0) == 900
                self.checkpoint_widget.versions.header().resizeSection(0, 480)
                QtTest.QTest.qWait(40)
                self.checkpoint_widget.window().grab().save(str(DIRECTORY / "ai-checkpoint.png"))
                self.geometry_checks = self.check_menu_toggles_and_geometry()
                self.stage = 20
                return
            elif self.stage == 20:
                try:
                    next(self.geometry_checks)
                except StopIteration:
                    if self.after_ui_checks():
                        self.finish()
                    else:
                        self.stage = 13  # opt-in paid scenarios, if explicitly selected
                return
            else:
                self.extra_tick()
                return
            self.stage += 1
        except Exception:
            self.finish(traceback.format_exc())


    def check_menu_toggles_and_geometry(self):
        def find(name):
            return next((pane for pane in hou.ui.paneTabs()
                         if pane.type() == hou.paneTabType.PythonPanel
                         and pane.activeInterface() is not None and pane.activeInterface().name() == name), None)
        for name, symbol, label, position, size in (
                ("loom_workspace", "h.loom_open_agent", "Toggle Loom", (90, 120), (910, 760)),
                ("loom_checkpoints", "h.loom_open_checkpoints", "Toggle Checkpoints", (350, 210), (780, 640))):
            pane = find(name)
            if pane is None:
                import loom
                from unittest.mock import patch
                module = loom if name == "loom_workspace" else loom.checkpoints
                original = module.toggle_pane
                called = []
                def invoke():
                    called.append(True)
                    return original()
                with patch.object(module, "toggle_pane", side_effect=invoke):
                    trigger_menu(symbol, label)
                    yield  # Native open/close work needs a real Houdini idle boundary, not nested qWait.
                assert called == [True], "Menu script did not invoke the toggle"
                pane = find(name)
            assert pane is not None, "Toggle did not open " + name
            floating = pane.floatingPanel()
            floating.setSize(size)
            floating.setPosition(position)
            yield
            expected = (tuple(floating.position()), tuple(floating.size()))
            trigger_menu(symbol, label)
            yield
            assert find(name) is None, "Toggle did not close " + name
            trigger_menu(symbol, label)
            yield
            pane = find(name)
            assert pane is not None, "Toggle did not reopen " + name
            actual = (tuple(pane.floatingPanel().position()), tuple(pane.floatingPanel().size()))
            assert actual == expected, repr((name, expected, actual))
            log("PASS menu toggle and restored geometry: " + repr((name, actual)))
        self.pane = find("loom_workspace")
        self.checkpoint_pane = find("loom_checkpoints")
        self.checkpoint_widget = self.checkpoint_pane.activeInterfaceRootWidget()

    def after_ui_checks(self):
        return True

    def extra_tick(self):
        raise AssertionError("Unexpected GUI test stage")


def main():
    global test
    log("Started isolated OFFLINE GUI test: " + hou.applicationVersionString())
    test = GuiSmokeTest()


if __name__ in ("__main__", "<run_path>"):
    main()

"""Offline Zed bridge tests. Never launch Zed, Houdini, or a provider."""
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
import importlib
import queue
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python3.13libs"))
import ko_zed as zed


class FakeParm:
    def __init__(self, text="initial", name="snippet", node_type="attribwrangle", tags=None):
        self.text = text
        self._name = name
        self._tags = tags or {}
        self._node = SimpleNamespace(sessionId=lambda: 42, name=lambda: "wrangle1",
                                     type=lambda: SimpleNamespace(name=lambda: node_type))
        self.keys = []
        self.locked = False
        self.writes = []

    def unexpandedString(self):
        return self.text

    def name(self):
        return self._name

    def path(self):
        return "/obj/geo/wrangle1/" + self._name

    def node(self):
        return self._node

    def parmTemplate(self):
        return SimpleNamespace(tags=lambda: self._tags, type=lambda: "string")

    def keyframes(self):
        return self.keys

    def isLocked(self):
        return self.locked

    def set(self, text, **kwargs):
        self.writes.append((text, kwargs))
        self.text = text


def fake_hou():
    return SimpleNamespace(
        parmTemplateType=SimpleNamespace(String="string"),
        undos=SimpleNamespace(group=lambda label: nullcontext()),
        ui=SimpleNamespace(setStatusMessage=Mock()),
        hipFileEventType=SimpleNamespace(BeforeLoad="load", BeforeClear="clear"))


class BindingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "code.vfl"
        self.path.write_text("initial", encoding="utf-8")
        self.parm = FakeParm()
        self.binding = zed.Binding(self.parm, self.path, "initial")
        self.apply = Mock(side_effect=lambda b, text: setattr(b.parm, "text", text))

    def save(self, text):
        self.path.write_text(text, encoding="utf-8")

    def poll(self):
        self.binding.poll(self.apply)

    def test_saved_code_applies_after_two_observations(self):
        text = 's@name = "$HIP/é"; // `literal`\n'
        self.save(text)
        self.poll()
        self.apply.assert_not_called()
        self.poll()
        self.assertEqual(self.parm.text, text)
        self.assertEqual(self.binding.baseline, text)
        self.poll()
        self.assertEqual(self.apply.call_count, 1)

    def test_concurrent_houdini_change_pauses_without_overwrite(self):
        self.parm.text = "edited in Houdini"
        self.save("edited in Zed")
        self.poll()
        self.poll()
        self.apply.assert_not_called()
        self.assertIn("Conflict", self.binding.paused)
        self.assertEqual(self.parm.text, "edited in Houdini")
        self.assertEqual(self.path.read_text(), "edited in Zed")

    def test_intermediate_save_is_not_applied(self):
        self.save("half")
        self.poll()
        self.save("complete")
        self.poll()
        self.apply.assert_not_called()
        self.poll()
        self.assertEqual(self.parm.text, "complete")

    def test_empty_code_is_valid(self):
        self.save("")
        self.poll()
        self.poll()
        self.assertEqual(self.parm.text, "")

    def test_atomic_save_gap_retries(self):
        self.path.unlink()
        self.poll()
        self.assertFalse(self.binding.paused)
        self.save("new")
        self.poll()
        self.poll()
        self.assertEqual(self.parm.text, "new")

    def test_manual_and_paused_bindings_do_not_apply(self):
        self.save("new")
        self.binding.auto = False
        self.poll()
        self.poll()
        self.binding.auto = True
        self.binding.paused = "conflict"
        self.poll()
        self.poll()
        self.apply.assert_not_called()

    def test_apply_error_pauses_and_preserves_baseline(self):
        self.save("new")
        self.apply.side_effect = ValueError("locked")
        self.poll()
        self.poll()
        self.assertEqual(self.binding.paused, "locked")
        self.assertEqual(self.binding.baseline, "initial")

    def test_size_limit(self):
        self.save("large")
        with patch.object(zed, "MAX_BYTES", 2):
            self.poll()
        self.assertIn("limit", self.binding.paused)
        self.apply.assert_not_called()

    def test_invalid_encoding_pauses(self):
        self.path.write_bytes(b"\xff")
        self.poll()
        self.assertTrue(self.binding.paused)
        self.apply.assert_not_called()


class RoutingTests(unittest.TestCase):
    def launcher(self, exists, run=None):
        launcher = zed.Launcher(Path("C:/workspace/Houdini-Zed-test"), "zed.exe", exists=exists,
                                run=run or Mock(return_value=SimpleNamespace(returncode=0)))
        self.addCleanup(launcher.close)
        return launcher

    def test_new_then_tabs_then_reopen(self):
        # Other unrelated windows do not affect this workspace-specific lookup.
        state = {"open": False}
        commands = []

        def run(command, **kwargs):
            commands.append(command)
            self.assertNotIn("--wait", command)
            self.assertNotIn("--add", command)
            self.assertEqual(kwargs["timeout"], 30)
            state["open"] = True
            return SimpleNamespace(returncode=0)

        launcher = self.launcher(lambda marker: state["open"], run)
        launcher._open(launcher.root / "one.vfl")
        launcher._open(launcher.root / "two.vfl")
        launcher._open(launcher.root / "one.vfl")
        state["open"] = False
        launcher._open(launcher.root / "three.py")
        self.assertEqual([c[1] for c in commands],
                         ["--new", "--existing", "--existing", "--existing", "--new", "--existing"])
        self.assertTrue(all(c[2] == str(launcher.root) for c in commands))
        self.assertTrue(all(len(c) == 3 for c in commands if c[1] == "--new"),
                        "Restore by stable workspace root only, before opening a new snippet")

    def test_open_does_not_wait_for_cli(self):
        entered, release = threading.Event(), threading.Event()

        def run(*args, **kwargs):
            entered.set()
            release.wait(2)
            return SimpleNamespace(returncode=0)

        launcher = self.launcher(lambda marker: True, run)
        self.addCleanup(release.set)
        launcher.open(launcher.root / "code.vfl")
        self.assertTrue(entered.wait(1))
        # open() already returned while the CLI remains blocked on release.
        self.assertFalse(release.is_set())
        release.set()

    def test_unknown_startup_does_not_spawn_duplicates(self):
        launcher = self.launcher(lambda marker: False)
        with patch.object(zed.time, "monotonic", side_effect=[0, 21]):
            with self.assertRaisesRegex(RuntimeError, "Cannot identify"):
                launcher._open(launcher.root / "one.vfl")
        with self.assertRaisesRegex(RuntimeError, "possible duplicate"):
            launcher._open(launcher.root / "two.vfl")
        self.assertEqual(launcher.run.call_count, 1)

    def test_cli_error_is_reported_on_queue(self):
        def run(command, stderr, **kwargs):
            stderr.write(b"CLI failed")
            return SimpleNamespace(returncode=1)

        launcher = self.launcher(lambda marker: True, run)
        launcher.open(launcher.root / "code.vfl")
        self.assertIn("CLI failed", launcher.errors.get(timeout=2))

    def test_cli_output_is_never_piped(self):
        # A cold start's Zed.exe inherits CLI handles; pipes would block until Zed exits.
        run = Mock(return_value=SimpleNamespace(returncode=0))
        self.launcher(lambda marker: True, run)._open(Path("code.vfl"))
        kwargs = run.call_args.kwargs
        self.assertNotIn("capture_output", kwargs)
        for stream in ("stdin", "stdout", "stderr"):
            self.assertNotEqual(kwargs.get(stream), zed.subprocess.PIPE)
            self.assertIsNotNone(kwargs.get(stream))


class HoudiniAdapterTests(unittest.TestCase):
    def setUp(self):
        self.hou = fake_hou()
        context = patch.dict(sys.modules, {"hou": self.hou})
        context.start()
        self.addCleanup(context.stop)

    def test_apply_never_follows_parameter_reference(self):
        parm = FakeParm()
        binding = zed.Binding(parm, Path("unused"), "initial")
        zed.Bridge.apply(binding, "$HIP/code")
        self.assertEqual(parm.writes, [("$HIP/code", {"follow_parm_reference": False})])

    def test_expression_or_locked_parameter_is_rejected(self):
        parm = FakeParm()
        parm.keys = ["expression"]
        with self.assertRaisesRegex(ValueError, "Expressions"):
            zed.Bridge.validate(parm)
        parm.keys = []
        parm.locked = True
        with self.assertRaisesRegex(ValueError, "locked"):
            zed.Bridge.validate(parm)

    def test_repeated_attach_preserves_pending_file(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge = zed.Bridge.__new__(zed.Bridge)
            bridge.root = Path(directory)
            bridge.bindings = {}
            bridge.launcher = Mock()
            parm = FakeParm()
            first = bridge.attach(parm)
            first.write_text("pending edit", encoding="utf-8")
            self.assertEqual(bridge.attach(parm), first)
            self.assertEqual(first.read_text(), "pending edit")
            self.assertEqual(len(bridge.bindings), 1)
            self.assertEqual(bridge.launcher.open.call_count, 2)
            bridge.scene_event("load")
            self.assertEqual(bridge.bindings, {})

    def test_sync_exports_literal_code_and_backs_up_saved_edits(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "code.vfl"
            path.write_text("Zed revision", encoding="utf-8")
            text = 's@path = "$HIP/é"; // `literal`\n'
            parm = FakeParm(text=text)
            binding = zed.Binding(parm, path, "initial", auto=False,
                                  paused="conflict", pending="Zed revision")
            zed.Bridge.reload_from_houdini(binding)
            self.assertEqual(path.read_text(encoding="utf-8"), text)
            backups = list(path.parent.glob("*.backup-*"))
            self.assertEqual(len(backups), 1)
            self.assertEqual(backups[0].read_text(), "Zed revision")
            self.assertEqual((binding.baseline, binding.pending, binding.paused), (text, None, ""))
            self.assertFalse(binding.auto)
            self.assertFalse(parm.writes)
            zed.Bridge.reload_from_houdini(binding)
            self.assertEqual(len(list(path.parent.glob("*.backup-*"))), 1)
            self.assertFalse(list(path.parent.glob("*.sync-*")))

    def test_sync_recovers_missing_file_and_empty_code(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "code.vfl"
            binding = zed.Binding(FakeParm(text=""), path, "initial")
            zed.Bridge.reload_from_houdini(binding)
            self.assertEqual(path.read_bytes(), b"")
            self.assertEqual(binding.baseline, "")

    def test_failed_export_preserves_file_and_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "code.vfl"
            path.write_text("saved", encoding="utf-8")
            binding = zed.Binding(FakeParm(), path, "baseline", paused="conflict")
            with patch.object(Path, "replace", side_effect=PermissionError("busy")):
                with self.assertRaises(PermissionError):
                    zed.Bridge.reload_from_houdini(binding)
            self.assertEqual(path.read_text(), "saved")
            self.assertEqual(binding.baseline, "baseline")
            self.assertEqual(binding.paused, "conflict")
            self.assertFalse(list(path.parent.glob("*.sync-*")))
            with patch.object(zed, "MAX_BYTES", 2):
                with self.assertRaisesRegex(ValueError, "limit"):
                    zed.Bridge.reload_from_houdini(binding)
            binding.parm.locked = True
            with self.assertRaisesRegex(ValueError, "locked"):
                zed.Bridge.reload_from_houdini(binding)

    def test_sync_ui_cancel_empty_and_partial_failure(self):
        self.hou.ui.displayMessage = Mock(return_value=1)
        self.hou.severityType = SimpleNamespace(Error="error")
        with patch.object(zed, "_bridge", None), patch.object(zed, "bridge") as create:
            zed.sync_to_zed()
            create.assert_not_called()
        bindings = [zed.Binding(FakeParm(), Path(name), "initial") for name in ("one", "two")]
        with patch.object(zed, "_bridge", SimpleNamespace(bindings=dict(enumerate(bindings)))), \
             patch.object(zed.Bridge, "reload_from_houdini") as reload:
            zed.sync_to_zed()
            reload.assert_not_called()
            self.hou.ui.displayMessage.return_value = 0
            reload.side_effect = [PermissionError("busy"), None]
            zed.sync_to_zed()
            self.assertEqual(reload.call_count, 2)
            self.assertIn("sync failed", bindings[0].paused)
            self.assertIn("1 editing file", self.hou.ui.setStatusMessage.call_args.args[0])

    def test_language_extensions(self):
        for node_type, name, tags, expected in [
            ("attribwrangle", "snippet", {}, ".vfl"),
            ("python", "python", {}, ".py"),
            ("apex::script", "code", {}, ".apex"),
            ("opencl", "kernel", {}, ".cl"),
            ("custom", "source", {"editorlang": "python"}, ".py"),
            ("custom", "source", {}, ".txt"),
        ]:
            with self.subTest(node_type=node_type):
                self.assertEqual(zed.extension_for(FakeParm(name=name, node_type=node_type, tags=tags)), expected)

    def test_reload_keeps_live_bridge(self):
        sentinel = object()
        old = zed._bridge
        try:
            zed._bridge = sentinel
            importlib.reload(zed)
            self.assertIs(zed._bridge, sentinel)
        finally:
            zed._bridge = old

    def test_menu_and_shelf_xml(self):
        menu = ET.parse(ROOT / "PARMmenu.xml").getroot()
        self.assertIsNotNone(menu.find("menu/scriptItem[@id='ko_edit_in_zed']"))
        shelf = ET.parse(ROOT / "toolbar/ko_dev.shelf").getroot()
        self.assertIsNone(menu.find("menu/scriptItem[@id='ko_zed_sessions']"))
        for name in ("ko_edit_in_zed", "ko_zed_sessions", "ko_zed_shortcuts"):
            self.assertIsNone(shelf.find(f"toolshelf/memberTool[@name='{name}']"))
            self.assertIsNone(shelf.find(f"tool[@name='{name}']"))
        self.assertIsNotNone(shelf.find("toolshelf/memberTool[@name='ko_zed_sync']"))
        tool = shelf.find("tool[@name='ko_zed_sync']")
        self.assertIn("ko_zed.sync_to_zed()", tool.findtext("script"))
        ET.parse(ROOT / "config/Icons" / (tool.get("icon") + ".svg"))
        main = ET.parse(ROOT / "MainMenuCommon.xml").getroot()
        submenu = main.find("menuBar/subMenu[@id='ko_menu']/subMenu[@id='ko_zed_menu']")
        self.assertEqual(submenu.findtext("label"), "Zed")
        for name in ("sync", "sessions", "shortcuts"):
            self.assertIsNotNone(submenu.find(f"scriptItem[@id='h.ko_zed_{name}']/scriptCode"))


if __name__ == "__main__":
    unittest.main(verbosity=2)

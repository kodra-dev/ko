"""Persistent workspace ownership tests. Real OS locks; no Houdini/Zed/provider."""
from pathlib import Path
from types import SimpleNamespace as NS
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python3.13libs"))
import ko_zed as zed


class WorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)

    def acquire(self, **kwargs):
        lease = zed.WorkspaceLease.acquire(self.base, **kwargs)
        self.addCleanup(lease.close)
        return lease

    def test_reuses_first_slot_after_release(self):
        first = self.acquire()
        root = first.root
        self.assertEqual(root.name, "Houdini-Zed-01")
        first.close()
        first.close()  # idempotent
        self.assertEqual(self.acquire().root, root)

    def test_live_bridges_are_isolated_and_lowest_free_slot_reused(self):
        first, second = self.acquire(), self.acquire()
        self.assertNotEqual(first.root, second.root)
        self.assertEqual(second.root.name, "Houdini-Zed-02")
        first.close()
        self.assertEqual(self.acquire().root.name, "Houdini-Zed-01")
        self.assertEqual(self.acquire().root.name, "Houdini-Zed-03")

    def test_existing_lock_file_is_not_a_stale_owner(self):
        root = self.base / "Houdini-Zed-01"
        root.mkdir()
        lock = root / ".ko-zed-owner.lock"
        lock.write_bytes(b"old metadata, not a live lock")
        lease = self.acquire()
        self.assertEqual(lease.root, root)
        lease.close()
        self.assertEqual(lock.read_bytes(), b"old metadata, not a live lock")

    def test_slot_limit_fails_without_stealing_live_ownership(self):
        self.acquire(limit=1)
        with self.assertRaisesRegex(RuntimeError, "All 1"):
            self.acquire(limit=1)

    def test_other_process_cannot_claim_live_slot(self):
        first = self.acquire()
        script = "import sys;sys.path.insert(0,sys.argv[1]);import ko_zed;lease=ko_zed.WorkspaceLease.acquire(sys.argv[2]);print(lease.root.name);lease.close()"
        result = subprocess.run([sys.executable, "-c", script, str(ROOT / "python3.13libs"), str(self.base)],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "Houdini-Zed-02")
        self.assertEqual(first.root.name, "Houdini-Zed-01")

    def test_process_death_releases_slot_without_pid_cleanup(self):
        marker = self.base / "claimed.txt"
        script = ("import sys,time;from pathlib import Path;sys.path.insert(0,sys.argv[1]);import ko_zed;"
                  "lease=ko_zed.WorkspaceLease.acquire(sys.argv[2]);"
                  "Path(sys.argv[3]).write_text(lease.root.name);time.sleep(60)")
        process = subprocess.Popen([sys.executable, "-c", script, str(ROOT / "python3.13libs"),
                                    str(self.base), str(marker)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        def cleanup():
            if process.poll() is None:
                process.kill()
            process.wait(timeout=5)
        self.addCleanup(cleanup)
        deadline = time.monotonic() + 10
        while not marker.exists() and process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(marker.exists(), "Child did not acquire a slot")
        self.assertEqual(marker.read_text(), "Houdini-Zed-01")
        self.assertEqual(self.acquire().root.name, "Houdini-Zed-02")
        process.kill()
        process.wait(timeout=5)
        # Windows can signal process termination just before lock cleanup finishes.
        deadline = time.monotonic() + 5
        while True:
            reclaimed = self.acquire()
            if reclaimed.root.name == "Houdini-Zed-01":
                break
            reclaimed.close()
            self.assertLess(time.monotonic(), deadline, "OS did not release the dead process's lock")
            time.sleep(0.02)
        self.assertEqual(reclaimed.root.name, "Houdini-Zed-01")

    def test_settings_and_old_snippets_are_not_overwritten(self):
        lease = self.acquire()
        zed.prepare_workspace(lease.root)
        settings = lease.root / ".zed/settings.json"
        self.assertIn('"preview_tabs"', settings.read_text())
        custom = '// customized by the artist\n{"preview_tabs":{"enabled":true}}\n'
        settings.write_text(custom, encoding="utf-8")
        old = lease.root / "old-snippet.vfl"
        old.write_text("saved recovery text", encoding="utf-8")
        lease.close()
        again = self.acquire()
        zed.prepare_workspace(again.root)
        self.assertEqual(settings.read_text(encoding="utf-8"), custom)
        self.assertEqual(old.read_text(), "saved recovery text")
        self.assertTrue((lease.root / ".ko-zed-owner.lock").is_file())

    def test_launcher_cancels_queue_but_retains_lease_until_inflight_cli_finishes(self):
        lease = self.acquire()
        entered, release = threading.Event(), threading.Event()
        calls = []
        def run(command, **kwargs):
            calls.append(command)
            entered.set()
            release.wait(5)
            return NS(returncode=0)
        launcher = zed.Launcher(lease.root, "fake-zed", exists=lambda marker: True, run=run, lease=lease)
        def cleanup():
            release.set()
            launcher.close()
            launcher.worker.join(timeout=5)
        self.addCleanup(cleanup)
        launcher.open(lease.root / "first.vfl")
        self.assertTrue(entered.wait(2))
        launcher.open(lease.root / "queued.vfl")
        launcher.close()
        self.assertEqual(self.acquire().root.name, "Houdini-Zed-02")
        with self.assertRaisesRegex(RuntimeError, "closed"):
            launcher.open(lease.root / "after-close.vfl")
        release.set()
        launcher.worker.join(timeout=5)
        self.assertFalse(launcher.worker.is_alive())
        self.assertEqual(len(calls), 1)
        self.assertEqual(self.acquire().root.name, "Houdini-Zed-01")

    def test_bridge_initialization_failure_releases_slot(self):
        fakehou = NS(expandString=lambda text: str(self.base), hipFile=NS(eventCallbacks=lambda: ()))
        with patch.dict(sys.modules, {"hou": fakehou, "hutil": NS(Qt=NS(QtCore=NS())),
                                     "hutil.Qt": NS(QtCore=NS())}), \
             patch.object(zed, "zed_executable", return_value="fake-zed"), \
             patch.object(zed, "prepare_workspace", side_effect=OSError("test initialization failure")), \
             patch.dict(zed.os.environ, {"KO_ZED_WORKSPACE": str(self.base)}):
            with self.assertRaisesRegex(OSError, "initialization failure"):
                zed.Bridge()
        self.assertEqual(self.acquire().root.name, "Houdini-Zed-01")

    def test_reopened_bridge_does_not_reattach_old_files(self):
        previous = self.acquire()
        old = previous.root / "old.vfl"
        old.write_text("old code", encoding="utf-8")
        previous.close()
        callbacks = []
        timer = NS(setInterval=Mock(), timeout=NS(connect=Mock()), start=Mock(), stop=Mock(), deleteLater=Mock())
        qt = NS(QtCore=NS(QTimer=lambda parent: timer))
        fakehou = NS(expandString=lambda text: str(self.base), qt=NS(mainWindow=lambda: None),
                     hipFile=NS(addEventCallback=callbacks.append, removeEventCallback=callbacks.remove,
                                eventCallbacks=lambda: tuple(callbacks)))
        with patch.dict(sys.modules, {"hou": fakehou, "hutil": NS(Qt=qt), "hutil.Qt": qt}), \
             patch.object(zed, "zed_executable", return_value="fake-zed"), \
             patch.dict(zed.os.environ, {"KO_ZED_WORKSPACE": str(self.base)}):
            bridge = zed.Bridge()
            try:
                self.assertEqual(bridge.root, previous.root)
                self.assertEqual(bridge.bindings, {})
                self.assertTrue(bridge.launcher.requests.empty())
                self.assertEqual(old.read_text(), "old code")
            finally:
                bridge.close()
                bridge.launcher.worker.join(timeout=5)
        self.assertEqual(callbacks, [])
        self.assertEqual(self.acquire().root.name, "Houdini-Zed-01")


if __name__ == "__main__":
    unittest.main(verbosity=2)

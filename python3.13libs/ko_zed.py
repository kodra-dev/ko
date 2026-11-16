"""Nonblocking Houdini parameter editing in a dedicated Zed window (Windows).

No hou/Qt imports at module load, so synchronization and routing are testable offline.
All Houdini calls run on the UI thread; the launcher thread only invokes the Zed CLI.
"""
from dataclasses import dataclass
from pathlib import Path
import ctypes
import errno
import os
import queue
import re
import shutil
import subprocess
import tempfile
import threading
import time
import uuid

MAX_BYTES = 4 * 1024 * 1024


def zed_executable():
    configured = os.environ.get("KO_ZED_EXE")
    if configured:
        if not Path(configured).is_file():
            raise RuntimeError("KO_ZED_EXE must be the path to the Zed CLI executable.")
        return configured
    found = shutil.which("zed") or shutil.which("zeditor")
    if not found:
        candidate = Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/Zed/bin/zed.exe"
        if candidate.is_file():
            found = str(candidate)
    if not found:
        raise RuntimeError("Zed CLI not found. Set KO_ZED_EXE to its executable path.")
    return found


def window_exists(marker):
    """Look only at top-level Zed window titles, never editor contents/process secrets."""
    if os.name != "nt":
        raise RuntimeError("Dedicated-window routing currently supports Windows only.")
    from ctypes import wintypes
    user32 = ctypes.WinDLL("user32", use_last_error=True)
    callback_type = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    user32.GetWindowTextLengthW.argtypes = [wintypes.HWND]
    user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    user32.IsWindowVisible.argtypes = [wintypes.HWND]
    user32.EnumWindows.argtypes = [callback_type, wintypes.LPARAM]
    found = []

    @callback_type
    def visit(hwnd, _):
        if user32.IsWindowVisible(hwnd):
            size = user32.GetWindowTextLengthW(hwnd)
            title = ctypes.create_unicode_buffer(size + 1)
            user32.GetWindowTextW(hwnd, title, size + 1)
            if marker in title.value and "zed" in title.value.lower():
                found.append(hwnd)
        return True

    if not user32.EnumWindows(visit, 0):
        raise ctypes.WinError(ctypes.get_last_error())
    if len(found) > 1:
        raise RuntimeError("Multiple Houdini Zed windows found. Close the duplicate workspace window.")
    return bool(found)


class WorkspaceLease:
    """Stable workspace slots; an OS file lock isolates concurrently live bridges.

    No PID files to go stale: the OS releases the lock even if Houdini crashes.
    The first free slot is reused, preserving Zed's workspace-specific dock layout.
    """
    def __init__(self, root, handle):
        self.root = root
        self.handle = handle

    @classmethod
    def acquire(cls, base, limit=32):
        base = Path(base).expanduser().resolve()
        base.mkdir(parents=True, exist_ok=True)
        for slot in range(1, limit + 1):
            root = base / f"Houdini-Zed-{slot:02d}"
            root.mkdir(exist_ok=True)
            handle = (root / ".ko-zed-owner.lock").open("a+b")
            try:
                handle.seek(0, os.SEEK_END)
                if handle.tell() == 0:
                    handle.write(b"\0")
                    handle.flush()
                handle.seek(0)
                if os.name == "nt":
                    import msvcrt
                    msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as error:
                handle.close()
                if error.errno in (errno.EACCES, errno.EAGAIN, errno.EDEADLK):
                    continue
                raise
            return cls(root, handle)
        raise RuntimeError(f"All {limit} Houdini Zed workspace slots are in use.")

    def close(self):
        if self.handle is not None:
            # Closing the descriptor releases its lock. Never unlink a lock file:
            # another process may already have it open while claiming this slot.
            self.handle.close()
            self.handle = None


def prepare_workspace(root):
    """Initialize only new workspace settings; preserve user edits on later opens."""
    settings = root / ".zed/settings.json"
    settings.parent.mkdir(parents=True, exist_ok=True)
    try:
        with settings.open("x", encoding="utf-8") as stream:
            stream.write('{"preview_tabs": {"enabled": false}}\n')
    except FileExistsError:
        pass


class Launcher:
    """Serialize startup/open requests off the Houdini thread, including cold starts."""
    def __init__(self, root, executable, exists=window_exists, run=subprocess.run, lease=None):
        self.root = Path(root)
        self.executable = executable
        self.exists = exists
        self.run = run
        self.errors = queue.Queue()
        self.requests = queue.Queue()
        self.uncertain = False
        self.lease = lease
        self.stopping = threading.Event()
        self.worker = threading.Thread(target=self._work, name="ko-zed-launcher", daemon=True)
        self.worker.start()

    def open(self, path):
        if self.stopping.is_set():
            raise RuntimeError("Zed launcher has been closed.")
        self.requests.put(Path(path))

    def _open(self, path):
        reuse = self.exists(self.root.name)
        if self.uncertain and not reuse:
            raise RuntimeError("Previous Zed startup could not be confirmed. Restore its workspace "
                               "name in the window title; refusing to create a possible duplicate.")
        if not reuse:
            self.uncertain = True
            # Restore using the stable root ONLY. Including a new snippet here
            # could change the roots used to look up Zed's saved workspace state.
            self._run([self.executable, "--new", str(self.root)])
            # Do not let another click race a cold startup and create a second window.
            deadline = time.monotonic() + 20
            while not self.stopping.is_set() and not self.exists(self.root.name):
                if time.monotonic() >= deadline:
                    raise RuntimeError("Cannot identify the Houdini Zed window. Keep the workspace "
                                       "folder name visible in Zed's window title, then retry.")
                time.sleep(0.1)
        self.uncertain = False
        # Pass the root along with the file to match the dedicated workspace,
        # rather than adding the snippet to whichever unrelated window has focus.
        self._run([self.executable, "--existing", str(self.root), str(path)])

    def _run(self, command):
        if self.stopping.is_set():
            return
        # Never pipe the CLI's output: on a cold start it spawns Zed.exe, which inherits
        # the handles, so a pipe would not reach EOF until the editor itself exits.
        # A file only requires the CLI process to exit.
        with tempfile.TemporaryFile() as stderr:
            result = self.run(command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                              stderr=stderr, timeout=30,
                              creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            if result.returncode:
                stderr.seek(0)
                message = stderr.read().decode("utf-8", "replace").strip()
                raise RuntimeError(message or f"Zed exited with {result.returncode}")

    def _work(self):
        try:
            while not self.stopping.is_set():
                path = self.requests.get()
                if path is None or self.stopping.is_set():
                    return
                try:
                    self._open(path)
                except Exception as error:
                    self.errors.put(str(error))
        finally:
            # Keep the slot claimed until any in-flight CLI request has finished.
            # This also prevents shutdown/re-enable from sharing a still-launching slot.
            if self.lease is not None:
                self.lease.close()

    def close(self):
        self.stopping.set()
        self.requests.put(None)


@dataclass
class Binding:
    parm: object
    path: Path
    baseline: str
    auto: bool = True
    paused: str = ""
    pending: str | None = None

    def disk_text(self):
        if self.path.stat().st_size > MAX_BYTES:
            raise ValueError("Code file exceeds the 4 MiB safety limit")
        return self.path.read_text(encoding="utf-8-sig")

    def poll(self, apply):
        if self.paused or not self.auto:
            return
        try:
            text = self.disk_text()
            if text == self.baseline:
                self.pending = None
                return
            # Two equal observations debounce atomic replace and multi-write saves.
            if text != self.pending:
                self.pending = text
                return
            if self.parm.unexpandedString() != self.baseline:
                self.paused = "Conflict: parameter changed in Houdini. Resolve in Zed Sessions."
                return
            apply(self, text)
            self.baseline = text
            self.pending = None
        except Exception as error:
            # Atomic-save rename gaps are transient; retry on the next tick.
            if isinstance(error, (FileNotFoundError, PermissionError)):
                return
            self.paused = str(error)


def extension_for(parm):
    tags = parm.parmTemplate().tags()
    lang = tags.get("editorlang", "").lower()
    node_type = parm.node().type().name().lower()
    if "opencl" in lang or "opencl" in node_type:
        return ".cl"
    if "apex" in lang or ("apex" in node_type and "script" in node_type):
        return ".apex"
    if "python" in lang or parm.name() == "python" or "python" in node_type:
        return ".py"
    if "vex" in lang or "wrangle" in node_type:
        return ".vfl"
    return ".txt"


def code_parms(node):
    import hou
    known = {"snippet", "python", "code", "script", "kernel", "kernelcode"}
    return [p for p in node.parms()
            if p.parmTemplate().type() == hou.parmTemplateType.String
            and (p.name() in known or p.parmTemplate().tags().get("editorlang")
                 or p.parmTemplate().tags().get("editor") == "1")]


class Bridge:
    def __init__(self):
        import hou
        from hutil.Qt import QtCore
        executable = zed_executable()
        base = Path(os.environ.get("KO_ZED_WORKSPACE", hou.expandString("$HOUDINI_USER_PREF_DIR/zed-editing")))
        lease = WorkspaceLease.acquire(base)
        self.root = lease.root
        self.bindings = {}
        self.launcher = None
        self.timer = None
        try:
            prepare_workspace(self.root)
            self.timer = QtCore.QTimer(hou.qt.mainWindow())
            self.timer.setInterval(350)
            self.timer.timeout.connect(self.tick)
            self.launcher = Launcher(self.root, executable, lease=lease)
            hou.hipFile.addEventCallback(self.scene_event)
            self.timer.start()
        except Exception:
            if self.scene_event in hou.hipFile.eventCallbacks():
                hou.hipFile.removeEventCallback(self.scene_event)
            if self.timer is not None:
                self.timer.stop()
                self.timer.deleteLater()
            if self.launcher is not None:
                self.launcher.close()
            else:
                lease.close()
            raise

    def scene_event(self, event):
        import hou
        if event in (hou.hipFileEventType.BeforeLoad, hou.hipFileEventType.BeforeClear):
            self.bindings.clear()

    def attach(self, parm):
        import hou
        if parm.parmTemplate().type() != hou.parmTemplateType.String:
            raise ValueError("Choose a string/code parameter.")
        self.validate(parm)
        key = (parm.node().sessionId(), parm.name())
        binding = self.bindings.get(key)
        if binding is None:
            text = parm.unexpandedString()  # Never eval: preserve $variables and backticks.
            name = re.sub(r"[^A-Za-z0-9_-]", "_", parm.node().name() + "_" + parm.name())[:100]
            path = self.root / (name + "-" + uuid.uuid4().hex[:8] + extension_for(parm))
            path.write_text(text, encoding="utf-8")
            binding = Binding(parm, path, text)
            self.bindings[key] = binding
        self.launcher.open(binding.path)
        hou.ui.setStatusMessage("Zed: opening " + parm.path())
        return binding.path

    @staticmethod
    def validate(parm):
        if parm.keyframes():
            raise ValueError("Expressions/keyframed parameters are not supported. Use a literal code string.")
        if parm.isLocked():
            raise ValueError("Parameter is locked.")

    @staticmethod
    def apply(binding, text):
        import hou
        Bridge.validate(binding.parm)
        with hou.undos.group("Apply code from Zed"):
            binding.parm.set(text, follow_parm_reference=False)

    @staticmethod
    def reload_from_houdini(binding):
        """Explicit Houdini-wins export; preserve the previous saved file for recovery."""
        Bridge.validate(binding.parm)
        text = binding.parm.unexpandedString()
        encoded = text.encode("utf-8")
        if len(encoded) > MAX_BYTES:
            raise ValueError("Code exceeds the 4 MiB safety limit")
        path = binding.path
        if not path.exists() or path.read_bytes() != encoded:
            if path.exists():
                backup = path.with_name(path.name + ".backup-" + uuid.uuid4().hex[:8])
                shutil.copy2(path, backup)
            # Atomic replacement avoids exposing a partial export to Zed's watcher.
            temporary = path.with_name(path.name + ".sync-" + uuid.uuid4().hex)
            try:
                temporary.write_bytes(encoded)
                temporary.replace(path)
            finally:
                temporary.unlink(missing_ok=True)
        binding.baseline, binding.pending, binding.paused = text, None, ""

    def tick(self):
        import hou
        while not self.launcher.errors.empty():
            hou.ui.setStatusMessage("Zed: " + self.launcher.errors.get_nowait(), severity=hou.severityType.Warning)
        for key, binding in list(self.bindings.items()):
            try:
                binding.parm.node().sessionId()
            except hou.ObjectWasDeleted:
                del self.bindings[key]
                continue
            previous = binding.paused
            binding.poll(self.apply)
            if binding.paused and binding.paused != previous:
                hou.ui.setStatusMessage("Zed: " + binding.paused, severity=hou.severityType.Warning)

    def manage(self):
        import hou
        bindings = list(self.bindings.values())
        if not bindings:
            hou.ui.displayMessage("No Zed editing sessions are attached.")
            return
        labels = [f"{b.parm.path()}  [{b.paused or ('auto' if b.auto else 'manual')}]" for b in bindings]
        selected = hou.ui.selectFromList(labels, exclusive=True, title="Zed Sessions")
        if not selected:
            return
        b = bindings[selected[0]]
        action = hou.ui.displayMessage(
            f"{b.parm.path()}\n{b.path}\n{b.paused}\n\nApply File overwrites the parameter. "
            "Reload Houdini overwrites the saved editing file (not unsaved Zed text).",
            buttons=("Apply File", "Reload Houdini", "Toggle Auto Apply", "Detach", "Cancel"),
            default_choice=4, close_choice=4)
        if action == 0:
            text = b.disk_text()
            self.apply(b, text)
            b.baseline, b.pending, b.paused = text, None, ""
        elif action == 1:
            self.reload_from_houdini(b)
        elif action == 2:
            b.auto = not b.auto
        elif action == 3:
            for key, value in list(self.bindings.items()):
                if value is b:
                    del self.bindings[key]

    def close(self):
        import hou
        self.timer.stop()
        self.timer.deleteLater()
        hou.hipFile.removeEventCallback(self.scene_event)
        self.launcher.close()
        self.bindings.clear()


# The existing Reload Ko shelf tool reloads ko_* modules. Preserve callbacks/state.
_bridge = globals().get("_bridge")


def bridge():
    global _bridge
    if _bridge is None:
        if os.name != "nt":
            raise RuntimeError("Dedicated Zed window routing currently supports Windows only.")
        _bridge = Bridge()
    return _bridge


def edit_parameter(parm):
    """Public entry point; call from Houdini's UI thread."""
    path = bridge().attach(parm)
    import sys
    shortcuts = sys.modules.get("ko_zed_shortcuts")
    if shortcuts is not None:
        shortcuts.direct_launch()
    return path


def menu_edit(kwargs):
    import hou
    try:
        for parm in kwargs.get("parms", []):
            edit_parameter(parm)
    except Exception as error:
        hou.ui.displayMessage(str(error), severity=hou.severityType.Error)


def shelf_edit():
    import hou
    try:
        nodes = hou.selectedNodes()
        if not nodes:
            raise ValueError("Select a code node, or right-click its code parameter → Edit in Zed.")
        for node in nodes:
            candidates = code_parms(node)
            if not candidates:
                raise ValueError(f"No code parameter detected on {node.path()}. Use the parameter context menu.")
            if len(candidates) > 1:
                chosen = hou.ui.selectFromList([p.description() + " (" + p.name() + ")" for p in candidates],
                                               exclusive=True, title="Choose code parameter")
                if not chosen:
                    continue
                candidates = [candidates[chosen[0]]]
            edit_parameter(candidates[0])
    except Exception as error:
        hou.ui.displayMessage(str(error), severity=hou.severityType.Error)


def sync_to_zed():
    """UI action: explicitly refresh all attached files, never guess from selection."""
    import hou
    if _bridge is None or not _bridge.bindings:
        hou.ui.displayMessage("No Zed editing sessions are attached. Use Edit in Zed or Alt+E first.")
        return
    bindings = list(_bridge.bindings.values())
    choice = hou.ui.displayMessage(
        f"Sync {len(bindings)} attached editing file(s) from Houdini to Zed?\n\n"
        "Houdini wins: changed saved files are backed up alongside the originals.\n"
        "Unsaved Zed buffers are not replaced; resolve Zed's reload/conflict prompt yourself.",
        buttons=("Sync", "Cancel"), default_choice=1, close_choice=1)
    if choice != 0:
        return
    synced = 0
    errors = []
    for binding in bindings:
        try:
            Bridge.reload_from_houdini(binding)
            synced += 1
        except Exception as error:
            # Keep failed sessions paused so stale file contents cannot auto-apply.
            binding.paused = "Houdini → Zed sync failed: " + str(error)
            errors.append(f"{binding.path.name}: {error}")
    hou.ui.setStatusMessage(f"Zed: synced {synced} editing file(s) from Houdini.")
    if errors:
        hou.ui.displayMessage("Some Zed files could not be synced:\n" + "\n".join(errors),
                              severity=hou.severityType.Error)


def sessions():
    import hou
    try:
        bridge().manage()
    except Exception as error:
        hou.ui.displayMessage(str(error), severity=hou.severityType.Error)


def shutdown():
    """Detach everything and stop callbacks, without closing Zed or deleting files."""
    global _bridge
    import sys
    shortcuts = sys.modules.get("ko_zed_shortcuts")
    if shortcuts is not None:
        shortcuts.disable()
    if _bridge is not None:
        _bridge.close()
        _bridge = None

"""Canonical Loom data location and conservative, whole-directory migration.

No credential parsing, copying, database rewrites, or provider process is involved.
Close old Loom/Codex processes before migrating; a locked rename fails in place.
"""
import os
from pathlib import Path


def _require_idle_history(legacy):
    for path in legacy.glob("workspaces/*/workspace.lock"):
        try:
            with path.open("r+b") as lock:
                if os.name == "nt":
                    import msvcrt
                    msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
                    msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            raise RuntimeError("Close all old Loom/Houdini panes before migrating: a checkpoint workspace is in use. "
                               "No data was moved.") from exc


def migrate_data_home(base):
    base = Path(base)
    root, legacy = base / "Loom", base / "ko-ai"
    if not legacy.exists():
        return root
    if root.exists():
        if root.resolve() == legacy.resolve():
            return root  # A manually configured compatibility link is harmless.
        if not any(legacy.iterdir()):
            return root
        if any(root.iterdir()):
            raise RuntimeError("Both Loom and legacy ko-ai data folders exist. Nothing was merged or overwritten. "
                               "Set LOOM_HOME to the data folder you intend to use before opening Loom.")
        root.rmdir()  # An empty destination has no user data to merge.
    _require_idle_history(legacy)
    try:
        legacy.rename(root)
    except OSError as exc:
        if root.exists() and not legacy.exists():
            return root  # Another launching process completed the same rename.
        raise RuntimeError("Loom could not migrate its legacy data folder. Close all old Loom/Houdini "
                           "and private Codex processes, then restart Houdini. The legacy data was not copied "
                           "or deleted. You can explicitly set LOOM_HOME to keep using that folder.") from exc
    return root


def data_root():
    override = os.environ.get("LOOM_HOME")
    if override:
        return Path(override).expanduser().resolve()
    base = Path(os.environ.get("LOCALAPPDATA", str(Path.home() / ".local/share")))
    legacy_override = os.environ.get("KO_AI_HOME")
    if legacy_override:
        custom = Path(legacy_override).expanduser().resolve()
        if custom != (base / "ko-ai").resolve():
            return custom
    return migrate_data_home(base)

"""One session-level controller shared by views. No Qt or agent imports."""
from .scene import Workspace, _hou


class Session:
    def __init__(self):
        self.workspace = Workspace()
        self.clients = set()
        self.turn_owner = None

    def require_idle(self):
        if self.turn_owner is not None:
            raise RuntimeError("Stop the active Loom turn before changing checkpoints.")

    def release(self, client):
        self.clients.discard(client)
        if self.turn_owner is client:
            self.turn_owner = None
        if not self.clients:
            self.workspace.close()
            if getattr(_hou().session, "_loom_checkpoint_session", None) is self:
                delattr(_hou().session, "_loom_checkpoint_session")


def acquire(client):
    # hou.session survives agent reloads. Controllers are not owned by widgets.
    session = getattr(_hou().session, "_loom_checkpoint_session", None)
    if session is None:
        session = Session()
        _hou().session._loom_checkpoint_session = session
    session.clients.add(client)
    return session

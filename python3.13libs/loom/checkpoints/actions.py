"""Optional menu extensions. Safe to register before any pane exists."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Action:
    id: str
    label: str
    callback: object
    selection_count: int = 1


# Keep extension registrations when implementation code is reloaded.
_actions = globals().get("_actions", {})


def register_action(id, label, callback, selection_count=1):
    """An existing ID is replaced, making extension reloads idempotent.

    callback(workspace, checkpoint_ids, parent_widget) is called on the UI thread.
    """
    _actions[id] = Action(id, label, callback, selection_count)


def unregister_action(id):
    _actions.pop(id, None)


def registered_actions():
    return tuple(_actions.values())

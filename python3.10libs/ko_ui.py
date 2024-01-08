import hou
from collections.abc import Callable

def reset_viewer_state(viewer):
    if viewer.currentState() != 'sopview':
        viewer.enterViewState()
        viewer.enterCurrentNodeState()

def menuize(items: list[str], itemToLabel: Callable[[str], str] = lambda item: item):
    """
    Convert a list of strings to a list of dictionaries with "label" and "name" keys.
    """
    labels = [itemToLabel(item) for item in items]
    return [item for pair in zip(labels, items) for item in pair]
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

# multiparm index is 1-based
def currentMultiparmIndex(parm: hou.Parm) -> int:
    """
    Get the current multiparm index of a multiparm instance.
    """
    if not parm.isMultiParmInstance():
        raise Exception("Parm is not a multiparm instance")
    return parm.multiParmInstanceIndices()[0]

def currentParm() -> hou.Parm:
    return hou.pwd().parm( hou.expandString('$CH') ) 

def siblingMultiParm(name: str) -> hou.Parm:
    """
    Get the multiparm instance of a sibling multiparm.
    """
    parm = currentParm()
    index = currentMultiparmIndex(parm)
    return hou.parm(name+str(index))

def evalSiblingMultiParm(name: str) -> int:
    """
    Evaluate a sibling multiparm instance.
    """
    parm = siblingMultiParm(name)
    return parm.eval()
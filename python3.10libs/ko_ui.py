import hou
from collections.abc import Callable

def reset_viewer_state(viewer):
    if viewer.currentState() != 'sopview':
        viewer.enterViewState()
        viewer.enterCurrentNodeState()

def setShadingMode(shading_type: hou.glShadingType, display_set_type: hou.displaySetType = hou.displaySetType.DisplayModel):
    pane = hou.ui.curDesktop().paneTabOfType(hou.paneTabType.SceneViewer)
    settings = pane.curViewport().settings()
    tmplset = settings.displaySet(display_set_type)
    tmplset.setShadedMode(shading_type)

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

def siblingMultiParm(name: str, component_index: int | None = None, current_parm: hou.Parm = None) -> hou.Parm:
    """
    Get the multiparm instance of a sibling multiparm.
    """
    parm = current_parm if current_parm else currentParm() 
    index = currentMultiparmIndex(parm)
    component_index = component_index if component_index != None else parm.componentIndex()
    return hou.parmTuple(name+str(index))[component_index]

def evalSiblingMultiParm(name: str) -> int:
    """
    Evaluate a sibling multiparm instance.
    """
    parm = siblingMultiParm(name)
    return parm.eval()
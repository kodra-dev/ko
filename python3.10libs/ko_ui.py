import hou

def reset_viewer_state(viewer):
    if viewer.currentState() != 'sopview':
        viewer.enterViewState()
        viewer.enterCurrentNodeState()
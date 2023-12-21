import hou
import apex
import ko_ui

def resetRig(kwargs):
    node = kwargs['node']
    parm = node.parm("animation")
    cur = parm.eval()
    if cur:
        tmp = hou.Geometry()
        apex.packFolderMerge(tmp, cur, "* ^/*.rig")  
        parm.set(tmp)

    viewers = [pt for pt in hou.ui.paneTabs() if pt.type() == hou.paneTabType.SceneViewer]
    for v in viewers:
        ko_ui.reset_viewer_state(v)


def isApexGraph(node):
    if not node or node.type() != hou.SopNode:
        return False

    geo = node.geometry()
    return  (geo != None
            and geo.findPointAttrib("callback") != None
            and geo.findPointAttrib("name") != None
            and geo.findPointAttrib("parms") != None)


def isPackedRig(node):
    if not node or node.type() != hou.SopNode:
        return False

    geo = node.geometry()
    return (geo != None and bool(geo.extractPackedPaths("*.rig")))




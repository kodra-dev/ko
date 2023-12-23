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
    if not node or not isinstance(node, hou.SopNode):
        return False

    geo = node.geometry()
    return  (geo != None
            and geo.findPointAttrib("callback") != None
            and geo.findPointAttrib("name") != None
            and geo.findPointAttrib("parms") != None)


def isPackedRig(node):
    if not node or not isinstance(node, hou.SopNode):
        return False

    geo = node.geometry()
    return (geo != None and bool(geo.extractPackedPaths("*.rig")))


def getFrameRange(geo):
    """
    Get the range of keyframes from packed animation data (channel primitives) 
    """
    min_start = 10000000
    max_end = -10000000
    for prim in geo.prims():
        if isinstance(prim, hou.ChannelPrim):
            start = int(prim.start())
            end = int(prim.end())
            min_start = min(min_start, start)
            max_end = max(max_end, end)
    return (min_start, max_end)




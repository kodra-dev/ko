import hou
import apex
import ko_ui
import ko_sop

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
    if node.type() == hou.nodeType(hou.sopNodeTypeCategory(), "apex::editgraph"):
        return True

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


def applyRigComponent(node, fun):
    """
    Call this in a Python SOP or Python-based SOP
    """
    node = hou.pwd()
    base_name = node.evalParm("basename")
    rig_path = f"/{base_name}.rig"
    skel_path = f"/{base_name}.skel"

    geo = node.geometry()

    rig_geo = geo.unpackFromFolder(rig_path)
    skel = geo.unpackFromFolder(skel_path)

    rig = apex.Graph()
    rig.loadFromGeometry(rig_geo)

    parms = ko_sop.nodeParmsToDict(node)

    if not "compname" in parms or not parms["compname"]:
        raise Exception("No component name specified!")

    fun(rig, skel, parms)

    rig.layout()

    new_rig_geo = hou.Geometry()
    rig.writeToGeometry(new_rig_geo)

    success = geo.packToFolder(rig_path, new_rig_geo)
    if not success:
        raise Exception(f"Failed to pack rig to {rig_path}!")

    hidden_group = geo.findPrimGroup("_3d_hidden_primitives")
    if hidden_group:
        hidden_group.add(geo.prims()[-1])

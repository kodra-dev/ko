import hou
import apex
import ko_ui
import ko_sop
import ko_rigutils as ru

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


def applyRigComponent(node, fun, require_compname=True):
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
    parms['pwd'] = node

    if require_compname and (not "compname" in parms or not parms["compname"]):
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


def unpackInputSkel(node, basename):
    node = hou.pwd()
    geo = node.inputGeometry(0)
    skel_path = f"/{basename}.skel"
    skel = geo.unpackFromFolder(skel_path)
    return skel

def unpackInputRig(node, basename):
    node = hou.pwd()
    geo = node.inputGeometry(0)
    rig_path = f"/{basename}.rig"
    rig_geo = geo.unpackFromFolder(rig_path)
    rig = apex.Graph()
    rig.loadFromGeometry(rig_geo)
    return rig


def menuScriptBlendshapeChannels(geo: hou.Geometry):
    if not geo.findGlobalAttrib("clipchannels"):
        raise Exception("No clipchannels attribute found! Add Character Blend Shape Channels SOP to the skeleton first.")
    value = geo.attribValue("clipchannels")
    if not value or not isinstance(value, dict):
        raise Exception("No clipchannels attribute found! Add Character Blend Shape Channels SOP to the skeleton first.")
    channels = value.keys()

    return ko_ui.menuize(channels)

def menuScriptControlChannels(rig: apex.Graph, separated: bool = False):
    parms = ru.getParmsNode(rig)
    ports = rig.getOutputPorts(parms)
    ports = [rig.portName(p) for p in ports if rig.portTypeName(p) == "Vector3" or rig.portTypeName(p) == "Float"]
    ports.sort()
    return ko_ui.menuize(ports)

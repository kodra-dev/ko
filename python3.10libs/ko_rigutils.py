import hou
import apex
import ko_math as kmath
import ko_utils as kou
from kinefx import utils as ku
from collections.abc import Callable

def getNode(rig: apex.Graph, pattern: str, must_exist: bool = True) -> int:
    nodes = rig.matchNodes(pattern)
    if not nodes:
        if must_exist:
            raise Exception(f"Pattern {pattern} doesn't match any nodes.")
        return -1
    if len(nodes) > 1:
        raise Exception(f"Pattern {pattern} matches multiple nodes.")
    return nodes[0]

def tfo(rig: apex.Graph, name: str, must_exist: bool = True) -> int:
    if name.startswith("parent#"):
        name = name.removeprefix("parent#")
        return getParentTfo(rig, tfo(rig, name), must_exist=must_exist)
    elif name.startswith("first#") or name.startswith("last#"):
        name = name.removeprefix("first#").removeprefix("last#")
        nodes = rig.matchNodes(f"%callback(TransformObject) & {name}")
        nodes.sort(key=lambda n: rig.nodeName(n))
        if name.startswith("first#"):
            return kou.firstFromList(nodes, default=-1,
                                     must_exist=must_exist, message_on_fail=f"Pattern {name} doesn't match any nodes.")
        else:
            return kou.lastFromList(nodes, default=-1,
                                    must_exist=must_exist, message_on_fail=f"Pattern {name} doesn't match any nodes.")
    return getNode(rig, f"%callback(TransformObject) & {name}", must_exist=must_exist)

def ac(rig: apex.Graph, name: str, must_exist: bool = True) -> int:
    return getNode(rig, f"%callback(AbstractControl) & {name}", must_exist=must_exist)

# Most of these Tfo functions support FkTransform too
def tfoRestLocal(rig: apex.Graph, tfo: int) -> hou.Matrix4:
    return rig.getNodeParms(tfo)["restlocal"] or hou.Matrix4(1)

def tfoRestTransform(rig: apex.Graph, tfo: int) -> hou.Matrix4:
    xform = tfoRestLocal(rig, tfo)
    parent = getParentTfo(rig, tfo, must_exist=False)
    visited = set()
    visited.add(tfo)
    while parent:
        if parent in visited:
            raise Exception(f"Node {rig.nodeName(tfo)} has a circular parent chain.")
        visited.add(parent)
        parent_xform = rig.getNodeParms(parent)["restlocal"] or hou.Matrix4(1)
        xform = xform * parent_xform
        parent = getParentTfo(rig, parent, must_exist=False)
    return xform

def setTfoRestTransform(rig: apex.Graph, tfo: int, xform: hou.Matrix4):
    parent = getParentTfo(rig, tfo, must_exist=False)
    if parent:
        parent_xform = tfoRestTransform(rig, parent)
        xform = xform * parent_xform.inverted()
    updateParms(rig, tfo, { "restlocal": xform })

def tfoAncestors(rig: apex.Graph, tfo: int) -> list[int]:
    ancestors = []
    parent = getParentTfo(rig, tfo, must_exist=False)
    visited = set()
    visited.add(tfo)
    while parent:
        if parent in visited:
            raise Exception(f"Node {rig.nodeName(tfo)} has a circular parent chain.")
        ancestors.append(parent)
        parent = getParentTfo(rig, parent, must_exist=False)
    return ancestors

def extractTfoChain(rig: apex.Graph, tfos: list[int]) -> list[dict]:
    chain = []
    unchecked = set(tfos)
    root = None # root is the true root of the whole skeleton, not the root of the chain
    leaf = None

    while unchecked:
        t = unchecked.pop()
        leaf = t
        while True:
            parent = getParentTfo(rig, t, must_exist=False)
            if parent is None:
                root = t
                break
            else:
                t = parent
                if t in unchecked:
                    unchecked.remove(t)

    unchecked = set(tfos)
    n = leaf
    while n:
        if n in unchecked:
            unchecked.remove(n)
        rest_local = tfoRestLocal(rig, n)
        chain.insert(0, {
            "node": n,
            "name": rig.nodeName(n),
            "rest_local": rest_local,
        })
        if n == root:
            break
        n = getParentTfo(rig, n, must_exist=False)

    if unchecked:
        raise Exception(f"Tfos don't form a chain: {', '.join(rig.nodeName(n) for n in unchecked)}")

    xform = hou.Matrix4(1)
    for i in range(len(chain)):
        xform = chain[i]["rest_local"] * xform
        chain[i]["xform"] = xform

    return list(c for c in chain if c["node"] in tfos)


def axesToRord(primary_axis: int, secondary_axis: int) -> int:
    """
    primary, tertiary, secondary
    xyz = 0
    xzy = 1
    yxz = 2
    yzx = 3
    zxy = 4
    zyx = 5
    """
    if primary_axis == 0:
        if secondary_axis == 2:
            return 0
        if secondary_axis == 1:
            return 1
    elif primary_axis == 1:
        if secondary_axis == 2:
            return 2
        if secondary_axis == 0:
            return 3
    elif primary_axis == 2:
        if secondary_axis == 0:
            return 4
        if secondary_axis == 1:
            return 5
    raise Exception(f"Invalid axes {primary_axis}, {secondary_axis}")

def unparentTfo(rig: apex.Graph, child: int):
    child_xform = tfoRestTransform(rig, child)
    updateParms(rig, child, { "restlocal": child_xform })
    parent = getParentTfo(rig, child, must_exist=False)
    if parent != -1:
        rig.removeWires(rig.portWires(getInPort(rig, child, "parent")), True)
        rig.removeWires(rig.portWires(getInPort(rig, child, "parentlocal")), True)

def setParentTfo(rig: apex.Graph, child: int, parent: int, compensate_xform: bool = False):
    if compensate_xform:
        child_xform = tfoRestTransform(rig, child)
        parent_xform = tfoRestTransform(rig, parent)
        child_xform = child_xform * parent_xform.inverted()
        updateParms(rig, child, { "restlocal": child_xform })
    connect(rig, parent, "xform", child, "parent")
    connect(rig, parent, "localxform", child, "parentlocal")


def getParentTfo(rig: apex.Graph, child: int, must_exist: bool = True) -> int:
    node = getSourceNode(rig, child, "parent", must_exist=must_exist)
    result = -1
    if node != -1:
        cb = rig.callbackName(node)
        if cb == "TransformObject" or cb == "rig::FkTransform":
            result = node
    if result != -1:
        return result
    if must_exist:
        raise Exception(f"Node {rig.nodeName(child)} has no parent.")
    return None

def restoreDefParentingIfTgt(rig: apex.Graph, tgt: int):
    tgt_name = rig.nodeName(tgt)
    if tgt_name.startswith("TGT_"):
        def_name = tgt_name.replace("TGT_", "DEF_")
        def_node = tfo(rig, def_name)
        setParentTfo(rig, def_node, tgt, compensate_xform=True)


def getOriginalParentTfo(rig: apex.Graph, skel: hou.Geometry, child: int, must_exist: bool = True) -> int:
    child_name = rig.nodeName(child)
    child_joint = ku.findPointName(skel, child_name)
    if not child_joint:
        raise Exception(f"Joint {child_name} doesn't exist.")
    parent_joint = ku.getPointParent(child_joint)
    if not parent_joint:
        if must_exist:
            raise Exception(f"Joint {child_name} has no parent.")
        return -1
    parent_name = parent_joint.attribValue("name")
    return tfo(rig, parent_name, must_exist=must_exist)

def computeLocalTransforms(skel: hou.Geometry) -> hou.Geometry:
    compute_local = hou.sopNodeTypeCategory().nodeVerb("kinefx::computetransform")
    compute_local.setParms({'mode': 1})
    return skel.freeze().execute(compute_local)

def getOriginalRestLocal(rig: apex.Graph, skel: hou.Geometry, child: int) -> hou.Matrix4:
    child_name = rig.nodeName(child)
    child_joint = ku.findPointName(skel, child_name)
    if not child_joint:
        raise Exception(f"Joint {child_name} doesn't exist.")
    return hou.Matrix4(child_joint.attribValue("localtransform"))

def getOriginalTransform(rig: apex.Graph, skel: hou.Geometry, p: int) -> hou.Matrix4:
    child_name = rig.nodeName(p)
    child_joint = ku.findPointName(skel, child_name)
    if not child_joint:
        raise Exception(f"Joint {child_name} doesn't exist.")
    transform = hou.Matrix3(child_joint.attribValue("transform"))
    position = child_joint.attribValue("P")
    return kmath.matrix3ToMatrix4(transform, position)
    


def insertBetweenParentTfo(rig: apex.Graph, child: int, parent: int,
                           biases: (float, float, float) = (1, 1, 1)):
    """
    Insert parent between child and its old parent
    biases: 0 if the new parent is at the old parent, 1 if at child (traslation, rotation, scale)
    """
    old_parent = getParentTfo(rig, child)
    setParentTfo(rig, parent, old_parent)

    child_parms = rig.getNodeParms(child)
    restlocal = child_parms["restlocal"] or hou.Matrix4(1)
    updateParms(rig, parent, {
        "restlocal": kmath.lerpMatrix4(hou.Matrix4(1), restlocal, biases),
        "xord": child_parms["xord"],
        "rord": child_parms["rord"],
    })
    setParentTfo(rig, child, parent, compensate_xform=True)


def getParmsNode(rig: apex.Graph) -> int:
    return getNode(rig, "%callback(__parms__)")

def getOutputNode(rig: apex.Graph) -> int:
    return getNode(rig, "%callback(__binding__)")

def getSetPointTransfromsNode(rig: apex.Graph) -> int:
    return getNode(rig, "%callback(SetPointTransforms) & pointtransform")

def setUpBlendshapeCoreNode(rig: apex.Graph, basename: str, node_storage: set[int]) -> int:
    op_core = safeAdd(rig, "ApplyBlendshapes", "sop::kinefx::characterblendshapescore",
                         node_storage=node_storage, get_existing=True)
    updateParms(rig, op_core, {
        "attribs": "P N tangentu",
    })

    parms_node = getParmsNode(rig)
    shp_port = getOutPort(rig, parms_node, f"{basename}.shp")
    rig.addWire(shp_port, getInPort(rig, op_core, "geoinput0"))
    bone_deform = getNode(rig, "Bonedeformation")
    connect(rig, op_core, "geo", bone_deform, "geoinput0")
    return op_core

def getLatestPosedSkelNode(rig: apex.Graph) -> int:
    node = getNode(rig, "%tag(latest_posed_skel)", must_exist=False)
    if node == -1:
        return getNode(rig, "pointtransform")
    return node

def updateLatestPosedSkelNode(rig: apex.Graph, new_node: int):
    n = getLatestPosedSkelNode(rig)
    removeNodeTag(rig, n, "latest_posed_skel")
    rig.setNodeTag(new_node, "latest_posed_skel")


def removeNodeTag(rig: apex.Graph, node: int, tag: str):
    tags = [t for t in rig.nodeTags(node)]
    arr = apex.StringArray()
    if tag in tags:
        for t in tags:
            if t != tag:
                arr.append(t)
        rig.setNodeTags(node, arr, True)

    

def promoteTfo(rig: apex.Graph, tfo: int, t: bool = True, r: bool = True, s: bool = False, demote: bool = False):
    parms_node = getParmsNode(rig)

    if demote:
        tp = getInPort(rig, tfo, "t")
        wires = rig.portWires(tp)
        rig.removeWires(wires, True)
        rp = getInPort(rig, tfo, "r")
        wires = rig.portWires(rp)
        rig.removeWires(wires, True)
        sp = getInPort(rig, tfo, "s")
        wires = rig.portWires(sp)
        rig.removeWires(wires, True)

    name = rig.nodeName(tfo)
    if t:
        rig.promoteInput(getInPort(rig, tfo, "t"), parms_node, f"{name}_t")
    if r:
        rig.promoteInput(getInPort(rig, tfo, "r"), parms_node, f"{name}_r")
    if s:
        rig.promoteInput(getInPort(rig, tfo, "s"), parms_node, f"{name}_s")

def promoteAc(rig: apex.Graph, ac: int, x: bool = True, y: bool = True, demote: bool = False):
    parms_node = getParmsNode(rig)

    if demote:
        xp = getInPort(rig, ac, "x")
        wires = rig.portWires(xp)
        rig.removeWires(wires, True)
        yp = getInPort(rig, ac, "y")
        wires = rig.portWires(yp)
        rig.removeWires(wires, True)

    name = rig.nodeName(ac)
    if x:
        rig.promoteInput(getInPort(rig, ac, "x"), parms_node, f"{name}_x")
    if y:
        rig.promoteInput(getInPort(rig, ac, "y"), parms_node, f"{name}_y")
    


def findSourceJoint(rig: apex.Graph, skel: hou.Geometry, node: int, must_exist: bool) -> int:
    props = rig.getNodeProperties(node)
    name = rig.nodeName(node)
    if props.contains("source_joint"):
        name = props["source_joint"]

    joint = ku.findPointName(skel, name)
    if joint == -1:
        if must_exist:
            raise Exception(f"Joint {name} doesn't exist.")
        return None
    
    return joint.number()



def inPortName(name: str) -> str:
    return name if name.endswith("[in]") else name + "[in]"

def outPortName(name: str) -> str:
    return name if name.endswith("[out]") else name + "[out]"
    
def getInPort(rig: apex.Graph, node: int, name: str, must_exist: bool = True) -> int | None:
    p = rig.getPort(node, inPortName(name))
    if p == -1 and must_exist:
        raise Exception(f"Node {rig.nodeName(node)} has no in port {name}")
    return p

def getOutPort(rig: apex.Graph, node: int, name: str, must_exist: bool = True) -> int | None:
    p = rig.getPort(node, outPortName(name))
    if p == -1 and must_exist:
        raise Exception(f"Node {rig.nodeName(node)} has no out port {name}")
    return p

def addNode(rig: apex.Graph, name: str, apex_cb: str, node_storage: set[int] | None = None) -> int:
    n = rig.addNode(name, apex_cb)
    if node_storage != None:
        node_storage.add(n)
    return n

def safeAdd(rig: apex.Graph, name: str, apex_cb: str, node_storage: set[int] | None = None, get_existing: bool = False) -> int:
    existing = getNode(rig, name, must_exist=False)
    n = -1
    if existing != -1:
        if get_existing:
            cb = rig.callbackName(existing)
            if cb == apex_cb:
                n = existing
            else:
                raise Exception(f"Node {name} already exists but with a different callback {cb}.")
        else:
            raise Exception(f"Node with name {name} already exists.")
    else:
        n = rig.addNode(name, apex_cb)
    if node_storage != None:
        node_storage.add(n)
    return n

def duplicateNode(rig: apex.Graph, node: int, name: str = "",
                  hijack_outputs: bool = False, node_storage: set[int] | None = None) -> int:
    if name == "":
        name = rig.nodeName(node) + "_copy"
    n = addNode(rig, name, rig.callbackName(node), node_storage)
    rig.setNodeParms(n, rig.getNodeParms(node))
    rig.setNodeProperties(n, rig.getNodeProperties(node))
    rig.setNodeTags(n, rig.nodeTags(node), False)
    # No API can get the node color

    in_ports = rig.getInputPorts(node)
    for ip in in_ports:
        ops = rig.connectedPorts(ip)
        if ops:
            rig.addWire(ops[0], getInPort(rig, n, rig.portName(ip)))
    
    if hijack_outputs:
        out_ports = rig.getOutputPorts(node)
        for op in out_ports:
            ips = rig.connectedPorts(op)
            for ip in ips:
                rig.addWire(getOutPort(rig, n, rig.portName(op)), ip)
    
    return n
    

def connect(rig: apex.Graph, src_node: int, src_port_name: str, dst_node: int, dst_port_name: str):
    sp = getOutPort(rig, src_node, outPortName(src_port_name))
    dp = getInPort(rig, dst_node, inPortName(dst_port_name))
    rig.addWire(sp, dp)

def connectToSub(rig: apex.Graph, src_node: int, src_port_name: str, dst_node: int, dst_port_name: str, sub_port_name: str = None):
    if sub_port_name == None:
        sub_port_name = rig.nodeName(src_node) + "_" + src_port_name
    sp = getOutPort(rig, src_node, outPortName(src_port_name))
    super_port = getInPort(rig, dst_node, inPortName(dst_port_name))
    sub_port = rig.addSubPort(super_port, sub_port_name)
    rig.addWire(sp, sub_port)

def connectFromSub(rig: apex.Graph, src_node: int, src_port_name: str, dst_node: int, dst_port_name: str, sub_port_name: str = None):
    if sub_port_name == None:
        sub_port_name = rig.nodeName(dst_node) + "_" + dst_port_name
    dp = getInPort(rig, dst_node, inPortName(dst_port_name))
    super_port = getOutPort(rig, src_node, outPortName(src_port_name))
    sub_port = rig.addSubPort(super_port, sub_port_name)
    rig.addWire(sub_port, dp)

def connectSubs(rig: apex.Graph, src_node: int, src_port_name: str, dst_node: int, dst_port_name: str,
                src_sub_port_name: str = None, dst_sub_port_name: str = None):
    if src_sub_port_name == None:
        src_sub_port_name = rig.nodeName(dst_node) + "_" + dst_port_name
    if dst_sub_port_name == None:
        dst_sub_port_name = rig.nodeName(src_node) + "_" + src_port_name
    sp = getOutPort(rig, src_node, outPortName(src_port_name))
    dp = getInPort(rig, dst_node, inPortName(dst_port_name))
    src_sub_port = rig.addSubPort(sp, src_sub_port_name)
    dst_sub_port = rig.addSubPort(dp, dst_sub_port_name)
    rig.addWire(src_sub_port, dst_sub_port)
    

def getSourcePort(rig: apex.Graph, dst_node: int, dst_port_name: str, must_exist: bool = True) -> int:
    port = getInPort(rig, dst_node, dst_port_name)
    srcPorts = rig.connectedPorts(port)
    return kou.firstFromList(srcPorts, default=-1, must_exist=must_exist,
                             message_on_fail=f"Node {rig.nodeName(dst_node)}'s port {dst_port_name} has no source connected.")

# Get source node of a destination node's port
def getSourceNode(rig: apex.Graph, dst_node: int, dst_port_name: str, must_exist: bool = True) -> int:
    port = getSourcePort(rig, dst_node, dst_port_name, must_exist)
    if port == -1:
        return -1
    return rig.portNode(port)

# Get destination node of a source node's port
def getDestNodes(rig: apex.Graph, src_node: int, src_port_name: str, criteria: Callable[[int], bool] = None) -> list[int]:
    port = getOutPort(rig, src_node, src_port_name)
    dstPorts = rig.connectedPorts(port)
    nodes = list(rig.portNode(p) for p in dstPorts)
    return list(n for n in nodes if criteria == None or criteria(n))

def getDestNode(rig: apex.Graph, src_node: int, src_port_name: str, criteria: Callable[[int], bool] = None,
                must_exist: bool = True) -> int:
    nodes = getDestNodes(rig, src_node, src_port_name, criteria)
    return kou.firstFromList(nodes, default=-1, must_exist=must_exist,
                             message_on_fail=f"Node {rig.nodeName(src_node)}'s port {src_port_name} has no destination connected.")

def unconnectAllInputs(rig: apex.Graph, node: int):
    ports = rig.getInputPorts(node)
    for p in ports:
        wires = rig.portWires(p)
        rig.removeWires(wires, True)
        for sp in rig.subPorts(p):
            wires = rig.portWires(sp)
            rig.removeWires(wires, True)


def updateParms(rig: apex.Graph, node: int, parms: dict):
    old_parms = rig.getNodeParms(node)
    for k in parms:
        old_parms[k] = parms[k]
    rig.setNodeParms(node, old_parms)

def xformmask(t: bool = True, r: bool = True, s: bool = True) -> int:
    return (1 if t else 0) | (2 if r else 0) | (4 if s else 0)

def setNodesColor(rig: apex.Graph, nodes: set[int], color: hou.Vector3):
    if type(color) == hou.Color:
        color = kmath.rgbVector3(color) 
    for n in nodes:
        rig.setNodeColor(n, color)


# CTLF = Fine control; CTLB = Blendshape control; CTLK = Keypose control
JOINT_PREFIXES = ["CTL", "MCH", "DEF", "TGT", "INT", "CTLB", "CTLK", "UI"]
JOINT_SUFFIXES = ["L", "R", "l", "r"] 

def splitJointName(name: str) -> (str, str, str):
    prefix = main = suffix = None
    parts = name.split('_')
    if parts[0] in JOINT_PREFIXES:
        prefix = parts[0]
        parts.pop(0)
    if parts[-1] in JOINT_SUFFIXES:
        suffix = parts[-1]
        parts.pop(-1)
    main = "_".join(parts)

    return (prefix, main, suffix)

def mirroredJointName(name: str) -> str:
    parts = list(splitJointName(name))
    if parts[2] == "L":
        parts[2] = "R"
    elif parts[2] == "R":
        parts[2] = "L"
    elif parts[2] == "l":
        parts[2] = "r"
    elif parts[2] == "r":
        parts[2] = "l"
    else:
        return name
    return joinJointName(parts[0], parts[1], parts[2])
        
def joinJointName(prefix: str, main: str, suffix: str) -> str:
    parts = list(p for p in [prefix, main, suffix] if p)
    return "_".join(parts)

def jointSetPrefix(old_name: str, prefix: str) -> str:
    parts = splitJointName(old_name)
    return joinJointName(prefix, parts[1], parts[2])


def jointSetSuffix(old_name: str, suffix: str) -> str:
    parts = splitJointName(old_name)
    return joinJointName(parts[0], parts[1], suffix)

def mchJointNameFromCtl(ctl_name: str) -> str:
    parts = splitJointName(ctl_name)
    return joinJointName("MCH", parts[1], parts[2])

def mchJointName(comp_name: str, sub_prefix: str = "") -> str:
    parts = splitJointName(comp_name)
    return joinJointName("MCH", (sub_prefix + '_' if sub_prefix else '') + parts[1], parts[2])

def ctlJointName(comp_name: str, sub_prefix: str = "") -> str:
    parts = splitJointName(comp_name)

    return joinJointName("CTL", (sub_prefix + '_' if sub_prefix else '') + parts[1], parts[2])


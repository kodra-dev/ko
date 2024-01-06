import hou
import apex
import ko_math as kmath
from kinefx import utils as ku
from collections.abc import Callable

def getNode(rig: apex.Graph, pattern: str, must_exist: bool = True) -> int | None:
    nodes = rig.matchNodes(pattern)
    if not nodes:
        if must_exist:
            raise Exception(f"Pattern {pattern} doesn't match any nodes.")
        return None
    if len(nodes) > 1:
        raise Exception(f"Pattern {pattern} matches multiple nodes.")
    return nodes[0]

def tfo(rig: apex.Graph, name: str) -> int:
    if name.startswith("parent#"):
        name = name.removeprefix("parent#")
        return getParentTfo(rig, tfo(rig, name))
    elif name.startswith("first#"):
        name = name.removeprefix("first#")
        nodes = rig.matchNodes(f"%callback(TransformObject) & {name}")
        nodes.sort(key=lambda n: rig.nodeName(n))
        return nodes[0]
    elif name.startswith("last#"):
        name = name.removeprefix("last#")
        nodes = rig.matchNodes(f"%callback(TransformObject) & {name}")
        nodes.sort(key=lambda n: rig.nodeName(n))
        return nodes[-1]
    return getNode(rig, f"%callback(TransformObject) & {name}")

def ac(rig: apex.Graph, name: str) -> int:
    return getNode(rig, f"%callback(AbstractControl) & {name}")

# Most of these Tfo functions support FkTransform too
def tfoRestLocal(rig: apex.Graph, tfo: int) -> hou.Matrix4:
    return rig.getNodeParms(tfo)["restlocal"] or hou.Matrix4(1)

def tfoRestTransform(rig: apex.Graph, tfo: int) -> hou.Matrix4:
    xform = tfoRestLocal(rig, tfo)
    parent = getParentTfo(rig, tfo, must_exist=False)
    while parent:
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
    while parent:
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


def insertBetweenParentTfo(rig: apex.Graph, child: int, parent: int,
                           biases: (float, float, float) = (1, 1, 1)):
    """
    Insert parent between child and its old parent
    biases: 0 if the new parent is at the old parent, 1 if at child (traslation, rotation, scale)
    """
    old_parent = getParentTfo(rig, child)
    setParentTfo(rig, parent, old_parent)

    restlocal = rig.getNodeParms(child)["restlocal"] or hou.Matrix4(1)
    updateParms(rig, parent, { "restlocal": kmath.lerpMatrix4(hou.Matrix4(1), restlocal, biases) })
    setParentTfo(rig, child, parent, compensate_xform=True)


def getParmsNode(rig: apex.Graph) -> int:
    return getNode(rig, "%callback(__parms__)")
    

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

def safeAdd(rig: apex.Graph, name: str, apex_cb: str, node_storage: set[int] | None = None) -> int:
    if(rig.matchNodes(name)):
        raise Exception(f"Node with name {name} already exists.")
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

def getSourcePort(rig: apex.Graph, dst_node: int, dst_port_name: str, must_exist: bool = True) -> int:
    port = getInPort(rig, dst_node, dst_port_name)
    srcPorts = rig.connectedPorts(port)
    if not srcPorts:
        if must_exist:
            raise Exception(f"Node {rig.nodeName(dst_node)}'s port {dst_port_name} has no source connected.")
        return -1
    return srcPorts[0]

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
    if not nodes:
        if must_exist:
            raise Exception(f"Node {rig.nodeName(src_node)}'s port {src_port_name} has no destination connected.")
        return -1
    return nodes[0]


def updateParms(rig: apex.Graph, node: int, parms: dict):
    old_parms = rig.getNodeParms(node)
    for k in parms:
        old_parms[k] = parms[k]
    rig.setNodeParms(node, old_parms)

def xformmask(t: bool = True, r: bool = True, s: bool = True) -> int:
    return (1 if t else 0) | (2 if r else 0) | (4 if s else 0)

def setNodesColor(rig: apex.Graph, nodes: set[int], color: hou.Color):
    for n in nodes:
        rig.setNodeColor(n, color)


JOINT_PREFIXES = ["CTL", "MCH", "DEF", "TGT"]
JOINT_SUFFIXES = ["L", "R"] 

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
    return joinJointName("MCH", sub_prefix + '_' + parts[1], parts[2])

def ctlJointName(comp_name: str, sub_prefix: str = "") -> str:
    parts = splitJointName(comp_name)

    return joinJointName("CTL", (sub_prefix + '_' if sub_prefix else '') + parts[1], parts[2])
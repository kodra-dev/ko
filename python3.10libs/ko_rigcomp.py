import hou
import apex
from kinefx import utils as ku
import ko_rigutils as ru
import ko_math as kmath

def setJointsRord(rig, skel, **kwargs):
    name_attrib = skel.findPointAttrib("name")
    rord_attrib = skel.findPointAttrib("rord")

    if not name_attrib:
        raise hou.NodeError("No name attribute")
    if not rord_attrib:
        raise hou.NodeError("No rord attribute")

    for pt in skel.points():
        name = pt.stringAttribValue(name_attrib)
        if name:
            joint = tfo(rig, name)
            parms = rig.getNodeParms(joint)
            parms['rord'] = pt.intAttribValue(rord_attrib)
            rig.setNodeParms(joint, parms)


def twoBoneIK(rig, skel, **kwargs):
    comp_name = kwargs['compname']
    color = kwargs['nodecolor']

    ctl_parent_name = kwargs['ctlparent']
    root_name = kwargs['root']
    mid_name = kwargs['mid']
    tip_name = kwargs['tip']
    # falloff = kwargs['falloff']
    # stretch_axis = kwargs['stretchaxis']

    promote_target_r = kwargs['promotargetr']
    promote_target_s = kwargs['promotargets']

    pole_length = kwargs['polelength']
    pole_style = kwargs['polestyle']

    root = ru.tfo(rig, root_name)
    mid = ru.tfo(rig, mid_name)
    tip = ru.tfo(rig, tip_name)
    root_joint = ku.findPointName(skel, root_name)
    mid_joint = ku.findPointName(skel, mid_name)
    tip_joint = ku.findPointName(skel, tip_name)
    root_joint_xform = ku.getPointTransform(root_joint)
    mid_joint_xform = ku.getPointTransform(mid_joint)
    tip_joint_xform = ku.getPointTransform(tip_joint)

    # mch_root: the root of the IK chain. e.g. hip
    mch_root_name = ru.mchJointName(comp_name, "IKRoot")
    mch_root = ru.safeAdd(rig, mch_root_name, "TransformObject")
    ru.insertBetweenParentTfo(rig, root, mch_root) 
    
    # ctl_target: the control of target joint. e.g. foot
    ctl_target_name = ru.ctlJoinName(comp_name, "IKTarget")
    ctl_target = ru.safeAdd(rig, ctl_target_name, "TransformObject")
    ru.updateParms(rig, ctl_target, { "restlocal": tip_joint_xform })
    ru.promoteTfo(rig, ctl_target, t=True, r=promote_target_r, s=promote_target_s)
    # TODO: restlocal needs to handle the case when ctl_parent is not origin
    if ctl_parent_name:
        ctl_parent = ru.getNode(rig, ctl_parent_name)
        ru.setParentTfo(rig, ctl_target, ctl_parent, compensate_xform=True)


    root_joint_xlate = root_joint_xform.extractTranslates()
    tip_joint_xlate = tip_joint_xform.extractTranslates()
    mid_joint_xlate = mid_joint_xform.extractTranslates()
    projected_mid = mid_joint_xlate.pointOnSegment(root_joint_xlate, tip_joint_xlate)

    ctl_ik_pole = None
    to_twist_xform = None

    if pole_style == "free":
        # Calculate ctl_ik_pole's position: project mid to the line root->tip,
        # then extend the line from projected_mid to mid by the length pole_length

        # ctl_ik_pole: the control of IK pole. e.g. knee's direction
        ctl_ik_pole_name = ru.ctlJoinName(comp_name, "IKPole")
        ctl_ik_pole = ru.safeAdd(rig, ctl_ik_pole_name, "TransformObject")

        pole_position = mid_joint_xlate + (mid_joint_xlate - projected_mid).normalized() * pole_length
        pole_xform = hou.hmath.buildTranslate(pole_position)

        ru.updateParms(rig, ctl_ik_pole, { "restlocal": pole_xform })
        ru.promoteTfo(rig, ctl_ik_pole, t=True, r=False, s=False)

        to_twist_xform = ru.getOutPort(rig, ctl_ik_pole, "xform")

        # TODO: restlocal needs to handle the case when ctl_parent is not origin
        if ctl_parent_name:
            ctl_parent = ru.getNode(rig, ctl_parent_name)
            ru.setParentTfo(rig, ctl_ik_pole, ctl_parent, compensate_xform=True)
    elif pole_style == "revolve":
        # In this case, pole is initially at the projected_mid, and will stay on the line root->tip dynamically
        # Its restlocal's orientation matters, but not its translation
        # ctl_ik_pole: the control of IK pole. e.g. knee's direction

        ctl_ik_pole_name = ru.ctlJoinName(comp_name, "IKPolePivot")
        ctl_ik_pole = ru.safeAdd(rig, ctl_ik_pole_name, "TransformObject")

        ru.setParentTfo(rig, ctl_ik_pole, mch_root)

        pole_lookat_pos = mid_joint_xlate + (mid_joint_xlate - projected_mid).normalized() * 1 # pole_length doesn't matter here
        pole_up_pos = root_joint_xlate
        pole_xform = kmath.lookat(pole_lookat_pos-projected_mid, pole_up_pos-projected_mid, projected_mid)
        pole_xform_local = pole_xform * root_joint_xform.inverted()
        ru.updateParms(rig, ctl_ik_pole, { "restlocal": pole_xform_local })
        ru.promoteTfo(rig, ctl_ik_pole, t=False, r=True, s=False)

        op_mul = rig.addNode("", "Multiply<Vector3,Matrix4>")
        ru.updateParms(rig, op_mul, { "a": hou.Vector3(0, 0, 1) })
        ru.connect(rig, ctl_ik_pole, "xform", op_mul, "b")
        op_build = rig.addNode("", "transform::Build")
        ru.connect(rig, op_mul, "result", op_build, "t")
        to_twist_xform = ru.getOutPort(rig, op_build, "m")

        mid_ratio = (projected_mid - root_joint_xlate).length() / (tip_joint_xlate - root_joint_xlate).length()
        op_slerp = rig.addNode("", "transform::Slerp<Matrix4>")
        ru.connect(rig, mch_root, "xform", op_slerp, "a")
        ru.connect(rig, ctl_target, "xform", op_slerp, "b")
        ru.updateParms(rig, op_slerp, { "bias": mid_ratio })
        ru.connect(rig, op_slerp, "result", ctl_ik_pole, "xform")
        ru.updateParms(rig, ctl_ik_pole, { "xformmask": ru.xformmask(t=True, r=False, s=False) })


    smooth_ik_name = f"SmoothIK_{comp_name}"
    smooth_ik = ru.safeAdd(rig, smooth_ik_name, "rig::TwoBoneIK")

    ru.connect(rig, mch_root, "xform", smooth_ik, "rootdriver")
    ru.connect(rig, ctl_target, "xform", smooth_ik, "goal")
    rig.addWire(to_twist_xform, ru.getInPort(rig, smooth_ik, "twist"))

    ru.connect(rig, smooth_ik, "rootout", root, "xform")
    ru.connect(rig, smooth_ik, "midout", mid, "xform")

    # We'd like to control tip's rotation with ctl_target, so we don't connect tipout to tip/xform here
    # ru.connect(rig, smooth_ik, "tipout", tip, "xform")
    ru.setParentTfo(rig, tip, ctl_target)
    ru.updateParms(rig, tip, { "restlocal": hou.Matrix4(1) })

    ru.updateParms(rig, smooth_ik, {
        "root": root_joint_xform,
        "mid": mid_joint_xform,
        "tip": tip_joint_xform,
        "blend": 1.0,
        "stretch": 1,
    })

    rig.setNodeColor(ctl_target, color)
    rig.setNodeColor(mch_root, color)
    rig.setNodeColor(ctl_ik_pole, color)
    rig.setNodeColor(smooth_ik, color)


def trackTo(rig, skel, **kwargs):
    comp_name = kwargs['compname']
    color = kwargs['nodecolor']

    driven_name = kwargs['driven']
    target_name = kwargs['target']
    mirroed = kwargs['mirrored']

    driven = ru.tfo(rig, driven_name)
    target = ru.tfo(rig, target_name)

    new_nodes = set()

    mch_ref_name = ru.mchJointName(comp_name, "TrackFrom")
    mch_ref = ru.safeAdd(rig, mch_ref_name, "TransformObject", new_nodes)
    ru.insertBetweenParentTfo(rig, driven, mch_ref)

    op_lookat = ru.addNode(rig, "lookat", "transform::LookAt", new_nodes)
    ru.connect(rig, mch_ref, "xform", op_lookat, "inxform")

    op_up = ru.addNode(rig, "up", "Value<Vector3>", new_nodes)
    ru.updateParms(rig, op_up, { "parm": hou.Vector3(0, 1, 0) })
    op_mul = ru.addNode(rig, "mul", "Multiply<Vector3,Matrix4>", new_nodes) 
    ru.connect(rig, op_up, "value", op_mul, "a")
    ru.connect(rig, mch_ref, "xform", op_mul, "b")
    op_build = ru.addNode(rig, "build", "transform::Build", new_nodes)
    ru.connect(rig, op_mul, "result", op_build, "t")

    ru.connect(rig, op_build, "m", op_lookat, "lookup")

    if mirroed:
        op_explode1 = ru.addNode(rig, "explode1", "transform::Explode", new_nodes)
        ru.connect(rig, target, "xform", op_explode1, "m")

        op_explode2 = ru.addNode(rig, "explode2", "transform::Explode", new_nodes)
        ru.connect(rig, mch_ref, "xform", op_explode2, "m")
        op_val = ru.addNode(rig, "val", "Value<Float>", new_nodes)
        ru.updateParms(rig, op_val, { "parm": 2.0 })
        op_mul2 = ru.addNode(rig, "mul2", "Multiply<Vector3,Float>", new_nodes)
        ru.connect(rig, op_explode2, "t", op_mul2, "a")
        ru.connect(rig, op_val, "value", op_mul2, "b")

        op_sub = ru.addNode(rig, "sub", "Subtract<Vector3>", new_nodes)
        ru.connect(rig, op_mul2, "result", op_sub, "a")
        ru.connect(rig, op_explode1, "t", op_sub, "b")

        op_build2 = ru.addNode(rig, "build2", "transform::Build", new_nodes)
        ru.connect(rig, op_sub, "result", op_build2, "t")

        ru.connect(rig, op_build2, "m", op_lookat, "lookat")

    else:
        ru.connect(rig, target, "xform", op_lookat, "lookat")

    ru.connect(rig, op_lookat, "xform", driven, "xform")

    ru.setNodesColor(rig, new_nodes, color)
    ru.promoteTfo(rig, target, t=True, r=False, s=False, demote=True)


def eyesTarget(rig, skel, **kwargs):
    trackTo(rig, skel,
            compname=kwargs['compname'] + "_L",
            nodecolor=kwargs['nodecolor'],
            driven=kwargs['driven'] + "_L",
            target=kwargs['target'] + "_L",
            mirrored=False)
    trackTo(rig, skel,
            compname=kwargs['compname'] + "_R",
            nodecolor=kwargs['nodecolor'],
            driven=kwargs['driven'] + "_R",
            target=kwargs['target'] + "_R",
            mirrored=True)
    target_center = ru.tfo(rig, kwargs['targetcenter'])
    ru.promoteTfo(rig, target_center, t=True, r=False, s=False, demote=True)


def rotationChain(rig, skel, **kwargs):
    compname = kwargs['compname']
    color = kwargs['nodecolor']

    control_locator_name = kwargs['controllocator']
    tfo_pattern = kwargs['tfopattern']

    primary_axis = kwargs['primaxis']
    secondary_axis = kwargs['scndaxis']
    lock_secondary_axis = kwargs['lockscndaxis']

    control_locator = ru.tfo(rig, control_locator_name)
    joints = rig.matchNodes(tfo_pattern)

    new_nodes = set()

    ctl_main_name = ru.ctlJoinName(compname, "Main")
    ctl_main = ru.safeAdd(rig, ctl_main_name, "TransformObject", new_nodes)
    rord = ru.axesToRord(primary_axis, secondary_axis)

    ancestors = ru.tfoAncestors(rig, control_locator)
    for j in joints:
        if j in ancestors:
            raise Exception(f"""Driven joint {rig.nodeName(j)} is an ancestor of control locator {rig.nodeName(control_locator)};
                            it's not allowed for now.""")

    parent = ru.getParentTfo(rig, control_locator, must_exist=False)
    if parent:
        ru.setParentTfo(rig, ctl_main, parent, compensate_xform=False)

    ru.updateParms(rig, ctl_main, {
        "restlocal": ru.tfoRestLocal(rig, control_locator),
        "rord": rord
    })
    ru.promoteTfo(rig, ctl_main, t=False, r=True, s=False)
    rig.setNodeTag(ctl_main, "rord_set")

    op_val = ru.addNode(rig, "val", "Value<Vector3>", new_nodes)
    value = hou.Vector3(0, 0, 0)
    for i in range(3):
        if i == primary_axis or (i == secondary_axis and not lock_secondary_axis):
            value[i] = 1.0 / len(joints)
        else:
            value[i] = 0
    ru.updateParms(rig, op_val, { "parm": value } )

    op_mul = ru.addNode(rig, "mul", "Multiply<Vector3>", new_nodes)
    ru.connect(rig, ctl_main, "r", op_mul, "a")
    ru.connect(rig, op_val, "value", op_mul, "b")

    joints.sort(key=lambda j: rig.nodeName(j))

    for i in range(len(joints)):
        joint = joints[i]
        mch_joint_name = ru.mchJointName(f"{compname}_{i}", "Int")
        mch_joint = ru.addNode(rig, mch_joint_name, "TransformObject", new_nodes)
        ru.updateParms(rig, mch_joint, { "rord": rord })
        ru.insertBetweenParentTfo(rig, joint, mch_joint)
        ru.connect(rig, op_mul, "result", mch_joint, "r")

    ru.setNodesColor(rig, new_nodes, color)


def twistChain(rig, skel, **kwargs):
    kwargs['primaxis'] = kwargs['twistaxis']
    kwargs['scndaxis'] = (kwargs['twistaxis'] + 1) % 3
    kwargs['lockscndaxis'] = True
    rotationChain(rig, skel, **kwargs)


def fbikChain(rig, skel, **kwargs):
    compname = kwargs['compname']
    color = kwargs['nodecolor']

    tfo_pattern = kwargs['tfopattern']
    iterations = kwargs['iterations']
    damping = kwargs['damping']
    tolerance = kwargs['tolerance']

    target_specs = kwargs['targets']

    new_nodes = set()

    joints = rig.matchNodes(tfo_pattern)
    chain = ru.extractTfoChain(rig, joints)

    op_geo = ru.safeAdd(rig, f"FBIKSkel_{compname}", "Value<Geometry>", new_nodes)
    last_add_joint = None
    last_geo_op = op_geo
    for i in range(len(chain)):
        n = chain[i]
        op_add_joint = ru.safeAdd(rig, f"AddJoint_{compname}_{i}", "skel::AddJoint", new_nodes)
        ru.updateParms(rig, op_add_joint, {
            "name": n["name"],
            "xform": n["xform"],
            "color": hou.Vector3(1, 1, 1),
        })
        if i == 0:
            ru.connect(rig, op_geo, "value", op_add_joint, "geo")
            last_geo_op = op_add_joint
        else:
            ru.connect(rig, last_geo_op, "geo", op_add_joint, "geo")
            op_set_parent = ru.addNode(rig, f"SetParent_{compname}_{i}", "skel::SetParent", new_nodes)
            ru.connect(rig, op_add_joint, "geo", op_set_parent, "geo")
            ru.connect(rig, op_add_joint, "ptnum", op_set_parent, "joint")
            ru.connect(rig, last_add_joint, "ptnum", op_set_parent, "parent")
            last_geo_op = op_set_parent
        last_add_joint = op_add_joint

    output = ru.getNode(rig, "output")
    outp = ru.getOutPort(rig, last_geo_op, "geo")
    inp = ru.getInPort(rig, output, "next")
    rig.setPortName(inp, "FBIKSkel")
    rig.addWire(outp, inp)

    op_skel = ru.safeAdd(rig, f"SkelFromGeo_{compname}", "fbik::SkeletonFromGeo", new_nodes)
    ru.connect(rig, last_geo_op, "geo", op_skel, "geo")
    op_solver = ru.safeAdd(rig, f"Solver_{compname}", "fbik::PhysIKSolver", new_nodes)
    ru.connect(rig, op_skel, "skel", op_solver, "skel")
    ru.updateParms(rig, op_solver, {
        "iterations": iterations,
        "damping": damping,
        "tolerance": tolerance,
    })

    last_solver_op = op_solver
    for ts in target_specs:
        name = ts['target#']
        weight = ts['weight#']
        priority = ts['priority#']
        ctl_name = ts['ctlname#']
        if not ctl_name:
            ctl_name = ru.ctlJoinName(f"{name}", "FBIKTarget")

        target = ru.tfo(rig, name)
        xform = ru.tfoRestTransform(rig, target)

        ctl = ru.safeAdd(rig, ctl_name, "TransformObject", new_nodes)
        ru.updateParms(rig, ctl, {
            "restlocal": xform,
        })
        ru.promoteTfo(rig, ctl, t=True, r=True, s=False)

        op_target = ru.safeAdd(rig, f"FBIKTarget_{compname}_{name}", "fbik::Target", new_nodes)
        ru.updateParms(rig, op_target, {
            "type": 2,
            "weight": weight,
            "priority": priority,
        });
        ru.connect(rig, ctl, "xform", op_target, "xform")

        op_set_target = ru.addNode(rig, f"SetTarget_{compname}_{name}", "fbik::SetTarget", new_nodes)
        ru.updateParms(rig, op_set_target, { "bone": name, })
        ru.connect(rig, last_solver_op, "solver", op_set_target, "solver")
        ru.connect(rig, op_target, "target", op_set_target, "target")
        last_solver_op = op_set_target

    op_solve = ru.addNode(rig, f"Solve_{compname}", "fbik::SolvePhysIK", new_nodes)
    ru.connect(rig, last_solver_op, "solver", op_solve, "solver")
    op_skel_geo = ru.addNode(rig, f"SkelToGeo_{compname}", "fbik::SkeletonUpdateGeo", new_nodes)
    ru.connect(rig, op_solve, "skel", op_skel_geo, "skel")
    ru.connect(rig, last_geo_op, "geo", op_skel_geo, "geo")

    for i in range(len(chain)):
        n = chain[i]
        op_get_xform = ru.addNode(rig, f"GetXform_{compname}_{i}", "skel::GetPointTransform", new_nodes)
        ru.updateParms(rig, op_get_xform, { "name": n["name"], })
        ru.connect(rig, op_skel_geo, "geo", op_get_xform, "geo")
        driven = ru.tfo(rig, n["name"])
        ru.connect(rig, op_get_xform, "xform", driven, "xform")


    ru.setNodesColor(rig, new_nodes, color)




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
    ctl_target_name = kwargs['ctltargetname']
    tip_follow_control = kwargs['tipfollowcontrol']
    # falloff = kwargs['falloff']
    # stretch_axis = kwargs['stretchaxis']

    promote_target_r = kwargs['promotargetr']
    promote_target_s = kwargs['promotargets']

    pole_length = kwargs['polelength']
    pole_style = kwargs['polestyle']
    
    new_nodes = set()

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
    mch_root = ru.safeAdd(rig, mch_root_name, "TransformObject", new_nodes)
    ru.insertBetweenParentTfo(rig, root, mch_root) 
    
    # ctl_target: the control of target joint. e.g. foot
    ctl_target = ru.tfo(rig, ctl_target_name)
    # ctl_target = ru.safeAdd(rig, ctl_target_name, "TransformObject")
    # ru.updateParms(rig, ctl_target, { "restlocal": tip_joint_xform })
    ru.promoteTfo(rig, ctl_target, t=True, r=promote_target_r, s=promote_target_s, demote=True)
    if ctl_parent_name:
        ctl_parent = ru.tfo(rig, ctl_parent_name)
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
        ctl_ik_pole_name = ru.ctlJointName(comp_name, "IKPole")
        ctl_ik_pole = ru.safeAdd(rig, ctl_ik_pole_name, "TransformObject", new_nodes)

        pole_position = mid_joint_xlate + (mid_joint_xlate - projected_mid).normalized() * pole_length
        pole_xform = hou.hmath.buildTranslate(pole_position)

        ru.updateParms(rig, ctl_ik_pole, { "restlocal": pole_xform })
        ru.promoteTfo(rig, ctl_ik_pole, t=True, r=False, s=False)

        to_twist_xform = ru.getOutPort(rig, ctl_ik_pole, "xform")

        if ctl_parent_name:
            ctl_parent = ru.tfo(rig, ctl_parent_name)
            ru.setParentTfo(rig, ctl_ik_pole, ctl_parent, compensate_xform=True)
    elif pole_style == "revolve":
        # In this case, pole is initially at the projected_mid, and will stay on the line root->tip dynamically
        # Its restlocal's orientation matters, but not its translation
        # ctl_ik_pole: the control of IK pole. e.g. knee's direction

        ctl_ik_pole_name = ru.ctlJointName(comp_name, "IKPolePivot")
        ctl_ik_pole = ru.safeAdd(rig, ctl_ik_pole_name, "TransformObject", new_nodes)

        ru.setParentTfo(rig, ctl_ik_pole, mch_root)

        pole_lookat_pos = mid_joint_xlate + (mid_joint_xlate - projected_mid).normalized() * 1 # pole_length doesn't matter here
        pole_up_pos = root_joint_xlate
        pole_xform = kmath.lookat(pole_lookat_pos-projected_mid, pole_up_pos-projected_mid, projected_mid)
        pole_xform_local = pole_xform * root_joint_xform.inverted()
        ru.updateParms(rig, ctl_ik_pole, { "restlocal": pole_xform_local })
        ru.promoteTfo(rig, ctl_ik_pole, t=False, r=True, s=False)

        op_mul = ru.addNode(rig, "mul", "Multiply<Vector3,Matrix4>", new_nodes)
        ru.updateParms(rig, op_mul, { "a": hou.Vector3(0, 0, 1) })
        ru.connect(rig, ctl_ik_pole, "xform", op_mul, "b")
        op_build = ru.addNode(rig, "build", "transform::Build", new_nodes)
        ru.connect(rig, op_mul, "result", op_build, "t")
        to_twist_xform = ru.getOutPort(rig, op_build, "m")

        mid_ratio = (projected_mid - root_joint_xlate).length() / (tip_joint_xlate - root_joint_xlate).length()
        op_slerp = ru.addNode(rig, "slerp", "transform::Slerp<Matrix4>", new_nodes)
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

    if tip_follow_control:
        ru.setParentTfo(rig, tip, ctl_target, compensate_xform=True)

    ru.connect(rig, smooth_ik, "rootout", root, "xform")
    ru.connect(rig, smooth_ik, "midout", mid, "xform")

    ru.updateParms(rig, smooth_ik, {
        "root": root_joint_xform,
        "mid": mid_joint_xform,
        "tip": tip_joint_xform,
        "blend": 1.0,
        "stretch": 1,
    })

    ru.setNodesColor(rig, new_nodes, color)


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

    ctl_name = kwargs['ctlname']
    tfo_pattern = kwargs['tfopattern']

    primary_axis = kwargs['primaxis']
    secondary_axis = kwargs['scndaxis']
    lock_secondary_axis = kwargs['lockscndaxis']

    joints = rig.matchNodes(tfo_pattern)

    new_nodes = set()

    ctl_main = ru.tfo(rig, ctl_name)
    rord = ru.axesToRord(primary_axis, secondary_axis)

    ancestors = ru.tfoAncestors(rig, ctl_main)
    for j in joints:
        if j in ancestors:
            raise Exception(f"""Driven joint {rig.nodeName(j)} is an ancestor of control {rig.nodeName(ctl_main)};
                            it's not allowed for now.""")

    ru.updateParms(rig, ctl_main, {
        "restlocal": ru.tfoRestLocal(rig, ctl_main),
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
        ru.insertBetweenParentTfo(rig, joint, mch_joint)
        # HACK: in case that the joint has its xform controlled by something else... like arm IK
        ip = ru.getInPort(rig, joint, "xform")
        op = ru.getSourcePort(rig, joint, "xform", must_exist=False)
        actual_mch = mch_joint
        if op != -1:
            rig.addWire(op, ru.getInPort(rig, mch_joint, "xform"))
            mch_joint_name2 = ru.mchJointName(f"{compname}_{i}", "Int2")
            mch_joint2 = ru.addNode(rig, mch_joint_name2, "TransformObject", new_nodes)
            ru.insertBetweenParentTfo(rig, joint, mch_joint2)
            rig.removeWires(rig.portWires(ip), True)
            actual_mch = mch_joint2
        ru.updateParms(rig, actual_mch, { "rord": rord })
        ru.connect(rig, op_mul, "result", actual_mch, "r")

    ru.setNodesColor(rig, new_nodes, color)


def twistChain(rig, skel, **kwargs):
    kwargs['primaxis'] = kwargs['twistaxis']
    kwargs['scndaxis'] = (kwargs['twistaxis'] + 1) % 3
    kwargs['lockscndaxis'] = True
    rotationChain(rig, skel, **kwargs)


def fbikChain(rig, skel, **kwargs):
    compname = kwargs['compname']
    color = kwargs['nodecolor']
    basename = kwargs['basename']

    ctl_parent_name = kwargs['ctlparent']
    tfo_pattern = kwargs['tfopattern']
    config_from_skel = kwargs['configfromskel']
    iterations = kwargs['iterations']
    damping = kwargs['damping']
    tolerance = kwargs['tolerance']

    target_specs = kwargs['targets']

    new_nodes = set()

    joints = rig.matchNodes(tfo_pattern)
    chain = ru.extractTfoChain(rig, joints)

    # Build up a temporary skeleton geo for FBIK
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

        if config_from_skel:
            pass # Not supported at this point, waiting for SideFX response
            # parms = ru.getParmsNode(rig)
            # skel_port = ru.getOutPort(rig, parms, f"{basename}.skel")
            # source_joint = ru.findSourceJoint(rig, skel, n["node"], True)
            # op_get = ru.addNode(rig, "", "geo::PointAttribValue<Dict>", new_nodes)
            # rig.addWire(skel_port, ru.getInPort(rig, op_get, "geo"))
            # ru.updateParms(rig, op_get, { "attribname": "fbik_jointconfig", "elemnum": source_joint })
            # op_set = ru.addNode(rig, "", "geo::SetPointAttribValue<Dict>", new_nodes)
            # ru.connect(rig, op_get, "value", op_set, "value")
            # ru.connect(rig, last_geo_op, "geo", op_set, "geo")
            # ru.connect(rig, op_add_joint, "ptnum", op_set, "elemnum")
            # ru.updateParms(rig, op_set, { "attribname": "fbik_jointconfig" })
            # last_geo_op = op_set


    output = ru.getNode(rig, "output")
    outp = ru.getOutPort(rig, last_geo_op, "geo")
    inp = ru.getInPort(rig, output, "next")
    rig.setPortName(inp, "FBIKSkel")
    rig.addWire(outp, inp)

    # Actual FBIK logic
    op_skel = ru.safeAdd(rig, f"SkelFromGeo_{compname}", "fbik::SkeletonFromGeo", new_nodes)
    ru.connect(rig, last_geo_op, "geo", op_skel, "geo")
    op_solver = ru.safeAdd(rig, f"Solver_{compname}", "fbik::PhysIKSolver", new_nodes)
    ru.connect(rig, op_skel, "skel", op_solver, "skel")
    ru.updateParms(rig, op_solver, {
        "iterations": iterations,
        "damping": damping,
        "tolerance": tolerance,
    })

    # Targets and their controls
    ctl_parent = None
    if ctl_parent_name:
        ctl_parent = ru.tfo(rig, ctl_parent_name)
    last_solver_op = op_solver
    for ts in target_specs:
        name = ts['target#']
        weight = ts['weight#']
        priority = ts['priority#']
        ctl_name = ts['ctlname#']
        type = ts['targettype#']
        if not ctl_name:
            ctl_name = ru.ctlJointName(f"{name}", "FBIKTarget")

        target = ru.tfo(rig, name)
        xform = ru.tfoRestTransform(rig, target)

        ctl = ru.safeAdd(rig, ctl_name, "TransformObject", new_nodes)
        ru.updateParms(rig, ctl, {
            "restlocal": xform,
        })
        ru.promoteTfo(rig, ctl, t=True, r=True, s=False)

        op_target = ru.safeAdd(rig, f"FBIKTarget_{compname}_{name}", "fbik::Target", new_nodes)
        ru.updateParms(rig, op_target, {
            "type": type,
            "weight": weight,
            "priority": priority,
        });
        ru.connect(rig, ctl, "xform", op_target, "xform")

        op_set_target = ru.addNode(rig, f"SetTarget_{compname}_{name}", "fbik::SetTarget", new_nodes)
        ru.updateParms(rig, op_set_target, { "bone": name, })
        ru.connect(rig, last_solver_op, "solver", op_set_target, "solver")
        ru.connect(rig, op_target, "target", op_set_target, "target")
        last_solver_op = op_set_target

        if ctl_parent:
            ru.setParentTfo(rig, ctl, ctl_parent, compensate_xform=True)


    op_solve = ru.addNode(rig, f"Solve_{compname}", "fbik::SolvePhysIK", new_nodes)
    ru.connect(rig, last_solver_op, "solver", op_solve, "solver")
    op_skel_geo = ru.addNode(rig, f"SkelToGeo_{compname}", "fbik::SkeletonUpdateGeo", new_nodes)
    ru.connect(rig, op_solve, "skel", op_skel_geo, "skel")
    ru.connect(rig, last_geo_op, "geo", op_skel_geo, "geo")

    # Update the Tfos from the updated skeleton geo
    for i in range(len(chain)):
        n = chain[i]
        op_get_xform = ru.addNode(rig, f"GetXform_{compname}_{i}", "skel::GetPointTransform", new_nodes)
        ru.updateParms(rig, op_get_xform, { "name": n["name"], })
        ru.connect(rig, op_skel_geo, "geo", op_get_xform, "geo")
        driven = ru.tfo(rig, n["name"])
        ru.connect(rig, op_get_xform, "xform", driven, "xform")

        # These tfos can't be manually controlled anymore, so we demote them
        ru.promoteTfo(rig, driven, t=False, r=False, s=False, demote=True)


    ru.setNodesColor(rig, new_nodes, color)


def torsoTwist(rig, skel, **kwargs):
    compname = kwargs['compname']
    color = kwargs['nodecolor']

    ctl_parent_name = kwargs['ctlparent']

    target_btm_name = kwargs['targetbtm']
    target_mid_name = kwargs['targetmid']
    target_top_name = kwargs['targettop']
    btm_weight = kwargs['btmweight']
    mid_weight = kwargs['midweight']
    top_weight = kwargs['topweight']

    mid_twist_ratio = kwargs['midtwistratio']
    btm_twist_ratio = kwargs['btmtwistratio']


    twist_axis = kwargs['twistaxis']

    fbik_kwargs = kwargs.copy()
    clt_btm_name = ru.ctlJointName(f"{compname}_Btm")
    ctl_mid_name = ru.ctlJointName(f"{compname}_Mid")
    clt_top_name = ru.ctlJointName(f"{compname}_Top")
    fbik_kwargs['targets'] = [
        { 'target#': target_btm_name, 'ctlname#': clt_btm_name,
          'weight#': btm_weight, 'priority#': 1, 'targettype#': 2 },
        { 'target#': target_mid_name, 'ctlname#': ctl_mid_name,
          'weight#': mid_weight, 'priority#': 1, 'targettype#': 2 },
        { 'target#': target_top_name, 'ctlname#': clt_top_name,
          'weight#': top_weight, 'priority#': 1, 'targettype#': 2 },
    ]

    fbikChain(rig, skel, **fbik_kwargs)

    new_nodes = set()

    ctl_name = ru.ctlJointName(compname)
    ctl_parent = None
    if ctl_parent_name:
        ctl_parent = ru.tfo(rig, ctl_parent_name)

    ctl = ru.safeAdd(rig, ctl_name, "TransformObject", new_nodes)
    ctl_btm = ru.tfo(rig, clt_btm_name)
    ctl_btm_xform = ru.tfoRestTransform(rig, ctl_btm)
    ctl_xlate = ctl_btm_xform.extractTranslates()
    ctl_xform = kmath.lookatAnyY(twist_axis, ctl_xlate)
    ru.setParentTfo(rig, ctl, ctl_parent)
    ru.setTfoRestTransform(rig, ctl, ctl_xform)
    ru.setParentTfo(rig, ctl_btm, ctl, compensate_xform=True)

    ctl_rest_local = ru.tfoRestLocal(rig, ctl)
    op_val = ru.addNode(rig, f"{ctl_name}_restlocal", "Value<Matrix4>", new_nodes)
    ru.updateParms(rig, op_val, { "parm": ctl_rest_local })
    op_mul0 = ru.addNode(rig, f"{ctl_name}_inherited", "Multiply<Matrix4>", new_nodes)
    ru.connect(rig, op_val, "value", op_mul0, "a")
    ru.connect(rig, ctl_parent, "xform", op_mul0, "b")
    op_invert = ru.addNode(rig, "invert", "Invert<Matrix4>", new_nodes)
    ru.connect(rig, op_mul0, "result", op_invert, "a")
    op_mul = ru.addNode(rig, "mul", "Multiply<Matrix4>", new_nodes)
    ru.connect(rig, op_invert, "result", op_mul, "a")
    ru.connect(rig, ctl, "xform", op_mul, "b")

    def mch_twist_logic(driven_ctl, ratio):
        mch_name = ru.mchJointNameFromCtl(rig.nodeName(driven_ctl))
        mch = ru.safeAdd(rig, mch_name, "TransformObject", new_nodes)
        ru.insertBetweenParentTfo(rig, driven_ctl, mch)
        mch_mid_rest_local = ru.tfoRestLocal(rig, mch)
        op_val2 = ru.addNode(rig, f"{mch_name}_restlocal", "Value<Matrix4>", new_nodes)
        ru.updateParms(rig, op_val2, { "parm": mch_mid_rest_local })
        op_mul2 = ru.addNode(rig, f"{mch_name}_inherited", "Multiply<Matrix4>", new_nodes)
        mch_parent = ru.getParentTfo(rig, mch)
        ru.connect(rig, op_val2, "value", op_mul2, "a")
        ru.connect(rig, mch_parent, "xform", op_mul2, "b")
        op_mul3 = ru.addNode(rig, "mul", "Multiply<Matrix4>", new_nodes)
        ru.connect(rig, op_mul2, "result", op_mul3, "a")
        op_lerp1 = ru.addNode(rig, f"{compname}_MidRatio", "Lerp<Matrix4>", new_nodes)
        ru.updateParms(rig, op_lerp1, {
            "a": hou.Matrix4(1),
            "bias": ratio,
        })
        ru.connect(rig, op_mul, "result", op_lerp1, "b")
        ru.connect(rig, op_lerp1, "result", op_mul3, "b")
        ru.connect(rig, op_mul3, "result", mch, "xform")

    # mch_btm: similar to mch_mid, but for ctl_btm
    ctl_mid = ru.tfo(rig, ctl_mid_name)
    ctl_btm = ru.tfo(rig, clt_btm_name)
    mch_twist_logic(ctl_mid, mid_twist_ratio)
    mch_twist_logic(ctl_btm, btm_twist_ratio)

    # fbik_target = ru.getDestNode(rig, ctl_mid, "xform", lambda n: rig.callbackName(n) == "fbik::Target")

    ctl_top = ru.tfo(rig, clt_top_name)
    ru.setParentTfo(rig, ctl_top, ctl, compensate_xform=True)

    ru.promoteTfo(rig, ctl, t=False, r=True, s=False)

    ru.setNodesColor(rig, new_nodes, color)


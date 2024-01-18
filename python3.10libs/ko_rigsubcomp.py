import ko_rigutils as ru

def mappingLogic(rig, value_in_port, value_out_port, driver_range, driven_range, use_ramp, ramp, new_nodes):
    driver_min = driver_range[0]
    driver_max = driver_range[1]
    driven_min = driven_range[0]
    driven_max = driven_range[1]

    if use_ramp:
        op_remap1 = ru.addNode(rig, f"remap1", "ko::Remap<Float>", new_nodes)
        rig.addWire(value_in_port, ru.getInPort(rig, op_remap1, "value"))
        ru.updateParms(rig, op_remap1, {
            'old_min': driver_min,
            'old_max': driver_max,
            'new_min': 0.0,
            'new_max': 1.0,
            'clamp': True,
        })
        
        ramp_interps = ["constant", "linear", "catmullrom", "monotonecubic", "bezier", "bspline", "hermite"]
        bases_str = ", ".join([
            f"\"{ramp_interps[ramp[i]['ramp#_#interp']]}\"" for i in range(len(ramp))
            ])
        values_str = ", ".join([str(ramp[i]['ramp#_#value']) for i in range(len(ramp))])
        poses_str = ", ".join([str(ramp[i]['ramp#_#pos']) for i in range(len(ramp))])
        vex_snippet =f"""
            string basis[] = array({bases_str});
            float value[] = array({values_str});
            float pos[] = array({poses_str});
            result = spline(basis, t, value, pos);
        """

        op_spline = ru.addNode(rig, f"spline", "RunVex", new_nodes)
        ru.updateParms(rig, op_spline, {
            "snippet": vex_snippet,
        })
        op_spline_inp = ru.getInPort(rig, op_spline, "inputs")
        op_spline_outp = ru.getOutPort(rig, op_spline, "outputs")
        t_p = rig.addSubPort(op_spline_inp, "t")
        remap1_result = ru.getOutPort(rig, op_remap1, "result")
        rig.addWire(remap1_result, t_p)

        op_remap2 = ru.addNode(rig, f"remap2", "ko::Remap<Float>", new_nodes)
        ru.updateParms(rig, op_remap2, {
            'old_min': 0.0,
            'old_max': 1.0,
            'new_min': driven_min,
            'new_max': driven_max,
            'clamp': True,
        })
        result_p = rig.addSubPort(op_spline_outp, "result")
        remap2_value = ru.getInPort(rig, op_remap2, "value")
        rig.addWire(result_p, remap2_value)

        rig.addWire(rig.getOutPort(op_remap2, "result"), value_out_port)
    else:
        op_remap = ru.addNode(rig, f"remap", "ko::Remap<Float>", new_nodes)
        rig.addWire(value_in_port, ru.getInPort(rig, op_remap, "value"))
        ru.updateParms(rig, op_remap, {
            'old_min': driver_min,
            'old_max': driver_max,
            'new_min': driven_min,
            'new_max': driven_max,
            'clamp': True,
        })
        rig.addWire(ru.getOutPort(rig, op_remap, "result"), value_out_port)


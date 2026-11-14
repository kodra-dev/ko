"""Conservative post-edit layout/display defaults; existing positions stay put."""


def finish_edit(steps, created_paths):
    import hou
    result = {"arranged": [], "display": None, "warnings": []}
    new = [hou.node(path) for path in created_paths]
    new = [node for node in new if node is not None]
    full_layouts = {step["path"] for step in steps if step["op"] == "layout" and step.get("all")}
    parents = {node.parent() for node in new}
    for parent in parents:
        if parent.path() in full_layouts:
            continue
        pending = [node for node in new if node.parent() == parent]
        movable = set(pending)
        occupied = [node.position() for node in parent.children() if node not in movable]
        while pending:
            node = next((n for n in pending if not any(source in pending for source in n.inputs())), pending[0])
            pending.remove(node)
            try:
                inputs = [source for source in node.inputs() if source is not None]
                outputs = [out for out in node.outputs() if out not in movable]
                if inputs:
                    anchor = inputs[0].position()
                    x, y = anchor[0], anchor[1] - 1.5
                elif outputs:
                    anchor = outputs[0].position()
                    x, y = anchor[0], anchor[1] + 1.5
                else:
                    x = max((point[0] for point in occupied), default=-3) + 3
                    y = max((point[1] for point in occupied), default=0)
                for _ in range(200):
                    if not any(abs(x - point[0]) < 1.6 and abs(y - point[1]) < 1.1 for point in occupied):
                        break
                    x += 2
                node.setPosition(hou.Vector2(x, y))
                occupied.append(node.position())
                result["arranged"].append(node.path())
            except Exception as exc:
                result["warnings"].append("Could not arrange " + node.path() + ": " + str(exc))
    if not any(step["op"] == "flags" and "display" in step for step in steps):
        paths = list(created_paths)
        for step in steps:
            if step["op"] in ("set", "connect", "flags"):
                if step["path"] in paths:
                    paths.remove(step["path"])
                paths.append(step["path"])
        nodes = [hou.node(path) for path in paths]
        nodes = [node for node in nodes if node is not None and node.type().category() == hou.sopNodeTypeCategory()]
        nodes = [node for node in nodes if any(node.inputs()) or node.type().minNumInputs() == 0]
        candidates = set(nodes)
        endpoints = [node for node in nodes if not any(out in candidates for out in node.outputs())]
        if endpoints:
            node = endpoints[-1]
            # Keep a displayed downstream OUT when it already includes the edits.
            pending, seen = list(node.outputs()), {node}
            while pending and len(seen) < 20000:
                downstream = pending.pop(0)
                if downstream in seen or downstream.parent() != node.parent():
                    continue
                seen.add(downstream)
                if downstream.isDisplayFlagSet():
                    node = downstream
                    break
                pending.extend(downstream.outputs())
            try:
                node.setDisplayFlag(True)
                if not any(step["op"] == "flags" and "render" in step for step in steps):
                    node.setRenderFlag(True)
                result["display"] = node.path()
            except Exception as exc:
                result["warnings"].append("Could not display " + node.path() + ": " + str(exc))
    return result

"""Resolve and freeze a single-network cleanup request before inference starts."""
import json


def resolve_scope(hou):
    selected = list(hou.selectedNodes())
    if selected:
        parent = selected[0].parent()
        if any(node.parent() != parent for node in selected):
            raise ValueError("Select nodes in a single network to tidy.")
        nodes = set(selected)
        if len(selected) == 1:
            pending = list(selected)
            while pending:
                node = pending.pop()
                for neighbour in tuple(node.inputs()) + tuple(node.outputs()):
                    if neighbour is not None and neighbour.parent() == parent and neighbour not in nodes:
                        nodes.add(neighbour)
                        pending.append(neighbour)
    else:
        editors = [pane for pane in hou.ui.paneTabs()
                   if pane.type() == hou.paneTabType.NetworkEditor and pane.isCurrentTab()]
        if len(editors) != 1:
            raise ValueError("Select a node, or leave exactly one Network Editor visible to identify the network.")
        parent = editors[0].pwd()
        nodes = set(parent.children())
    if not nodes:
        raise ValueError("There are no nodes to tidy in this network.")
    return parent, nodes


def request_text(parent, nodes):
    return ("Tidy Network. Scope frozen at click time: "
            + json.dumps({"network": parent.path(), "nodes": sorted(node.path() for node in nodes)})
            + "\nInspect these nodes and their context first. Improve layout; shorten overly long names "
            "while keeping meaningful contextual names. Merge trivial nodes or split overloaded nodes only "
            "when demonstrably behavior-preserving and clearer; do not refactor just to change node counts. "
            "Preserve output, wiring at the scope boundary, flags, expressions, references, animation and public "
            "interfaces. Skip risky renames/refactors when references cannot be verified. Only edit scoped nodes "
            "and new replacement nodes in this network; do not recurse into child networks. "
            "Design the layout from the meaning and dependencies of the nodes, not just their SOP types. "
            "Use position {op,path,value:[x,y]} steps for explicit placement; do not rely on Houdini auto-layout. "
            "Prefer a tall, relatively narrow top-to-bottom network over a wide one, without forcing unrelated "
            "branches into one column. Assume rounded wiring style (do not change the user's wiring preference). "
            "Reserve at least 2.25 network units of horizontal width per node including its displayed name, "
            "plus a small gap between adjacent columns. Leave vertical clearance for rounded wire bends; "
            "avoid wire crossings, wires through labels, and overlaps with unselected nodes or annotations. "
            "Group nodes by contextual purpose; where dependencies permit, align parallel nodes with similar "
            "semantic roles horizontally, and stack sequential stages vertically. Inspect positions before "
            "and after placement and refine spacing; do not equate successful positioning with readability. "
            "Read existing node comments and compare them with actual parameters, code and wiring. Correct "
            "stale or incorrect comments using comment steps (empty text removes an obsolete comment); "
            "preserve useful accurate explanations and do not invent explanations when intent is uncertain. "
            "Add comments, sticky notes or network boxes only to explain genuinely difficult sections; "
            "avoid redundant decoration. " 
            "Verify behavior and report changes and anything intentionally skipped. Do not render or simulate.")


class TidyScope:
    def __init__(self, parent, nodes):
        self.parent = parent
        self.nodes = set(nodes)

    def check(self, step, hou):
        op = step["op"]
        if op in ("create", "sticky", "box"):
            if hou.node(step["parent"]) != self.parent:
                raise PermissionError("Tidy edits must stay in the chosen network.")
            if op == "box" and any(hou.node(path) not in self.nodes for path in step["nodes"]):
                raise PermissionError("Network boxes may only contain scoped nodes.")
        elif op == "layout":
            raise PermissionError("Tidy requires explicit position steps, not Houdini auto-layout.")
        elif hou.node(step["path"]) not in self.nodes:
            raise PermissionError("Tidy may only edit nodes in the frozen scope.")

    def created(self, node):
        self.nodes.add(node)

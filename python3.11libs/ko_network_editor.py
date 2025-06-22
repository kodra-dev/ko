import hou
import hdefereval
import ko_sop

def add_title_sticky_note(ref_node):
    note = ref_node.parent().createStickyNote()
    note.setText("Title") 
    note.setDrawBackground(False)
    note.setTextColor(hou.Color((0.8, 0.8, 0.8)))
    note.setTextSize(0.6)
    note.setSize(hou.Vector2((2, 1.2)))
    ref_pose = ref_node.position()
    note.setPosition(ref_pose + hou.Vector2((-0.15, 0.3)))

def add_comment_sticky_note(ref_node):
    note = ref_node.parent().createStickyNote()
    note.setText("Comment\nSection")
    note.setDrawBackground(False)
    note.setTextColor(hou.Color((0.6, 0.6, 0.6)))
    note.setTextSize(0.3)
    note.setSize(hou.Vector2(2, 1))
    ref_pose = ref_node.position()
    ref_size = ref_node.size()
    note.setPosition(ref_pose + hou.Vector2((-ref_size.x() / 2 - 1, -ref_size.y() / 2 - 0.25)))


def open_in_apex_editor(editor, node, reuse_existing=True):
    existing_panel = None
    for p in hou.ui.floatingPanels():
        if p.panes() and p.panes()[0].tabs() and p.panes()[0].tabs()[0].type() == hou.paneTabType.ApexEditor:
            existing_panel = p

    if reuse_existing and existing_panel:
        panel = existing_panel
    else:
        panel = hou.ui.curDesktop().createFloatingPanel(hou.paneTabType.ApexEditor, (20, 500), (1300, 900),
                                                                immediate=True)
    apex_editor = panel.panes()[0].tabs()[0]
    apex_editor.showNetworkControls(False)
    apex_editor.setPref("showmenu", "0")
    old_current_node = editor.currentNode()

    editor.setCurrentNode(node)
    apex_editor.setPin(False)
    apex_editor.setPwd(node.parent())
    # editor.setCurrentNode(old_current_node)
    home_all(apex_editor)


def unpack_open_in_apex_editor(editor, node):
    UNPACK_NODE_NAME = "_unpack_to_view"

    geo = node.geometry()
    parent = editor.pwd()
    unpack_node = parent.node(UNPACK_NODE_NAME)
    if(unpack_node and unpack_node.type() != hou.sopNodeTypeCategory().nodeType("unpackfolder")):
        hou.ui.displayMessage(f"Node {unpack_node.name()} exists, but not a Unpack Folder SOP. Aborted.",
                                severity=hou.severityType.Error)
        return
    
    if(not unpack_node):
        unpack_node = parent.createNode("unpackfolder", UNPACK_NODE_NAME,
                                        run_init_scripts=False, load_contents=True, exact_type_name=True)
    
    editorBounds = editor.visibleBounds()

    parm = unpack_node.parm("pattern")
    parm.set(geo.extractPackedPaths("*.rig")[0])

    parm = unpack_node.parm("unpack")
    parm.set(1)

    unpack_node.setInput(0, node, 0)
    parent_box = node.parentNetworkBox()
    if(parent_box):
        parent_box.addItem(unpack_node)
    unpack_node.moveToGoodPosition()
    editor.setVisibleBounds(editorBounds) # The network editor view still moves sometimes... not sure why

    open_in_apex_editor(editor, unpack_node, reuse_existing=True)


def open_in_floating_window(editor, node, reuse_existing=True):
    existing_panel = next((p for p in hou.ui.floatingPanels() if p.panes()[0].tabs()[0].type() == hou.paneTabType.NetworkEditor),
                          None)
    if reuse_existing and existing_panel:
        panel = existing_panel
    else:
        panel = hou.ui.curDesktop().createFloatingPanel(hou.paneTabType.NetworkEditor, (20, 550), (1200, 800),
                                                        immediate=True)

    if editor.linkGroup() == hou.paneLinkType.FollowSelection:
        editor.setPin(True)
    new_editor = panel.panes()[0].tabs()[0]
    new_editor.showNetworkControls(False)
    new_editor.setPref("showmenu", "0")
    new_editor.setPwd(node)
    home_all(new_editor)

def open_parent_in_floating_window(editor, node, reuse_existing=True):
    open_in_floating_window(editor, node.parent(), reuse_existing=reuse_existing)


def home_all(editor):
    items = (it for (it, _, _) in editor.networkItemsInBox(hou.Vector2(-2000, -2000), hou.Vector2(2000, 2000)))
    boxes = (editor.itemRect(it) for it in items if isinstance(it, hou.NetworkMovableItem))
    bounds = hou.BoundingRect(0, 0, 0.1, 0.1)
    for b in boxes:
        bounds.enlargeToContain(b)
    bounds.expand((0.5, 0.25))
    editor.setVisibleBounds(bounds)


def recookUpmostPythonNodes(editor, debug_log=False):
    """
    Recook all Python SOPs and Python-based nodes that don't have an ancestor Python SOP / Python-based node.
    """
    node = editor.pwd()
    queue = list(c for c in node.children() if c.inputs() == ())
    if node.isSubNetwork():
        sub_inputs = node.indirectInputs()
        queue.extend(sub_inputs)
    visited = set()
    ignored = set()
    while queue:
        node = queue.pop()
        if node in visited:
            continue
        visited.add(node)

        if debug_log:
            print(f"Visiting {node.path()}")

        if isinstance(node, hou.SubnetIndirectInput):
            queue.extend(node.outputs())
            continue

        if node.type() == hou.sopNodeTypeCategory().nodeType("python") or ko_sop.isPythonBased(node):
            if debug_log:
                print(f"Recooking {node.path()}")

            try:
                node.cook(force=True)
            except hou.OperationFailed:
                raise hou.Error(f"Failed to recook {node.path()}. Aborted.")

            for d in node.outputs():
                ignored.add(d)
        else:
            if node in ignored:
                continue
            for d in node.outputs():
                if all(i in visited for i in d.inputs() if i):
                    queue.append(d)


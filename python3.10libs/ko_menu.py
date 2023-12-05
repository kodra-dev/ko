import hou
import hdefereval

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

def open_in_apex_editor(editor, node):
    apex_editor = hou.ui.curDesktop().createFloatingPaneTab(hou.paneTabType.ApexEditor, (20, 550), (1200, 800),
                                                            immediate=True)
    old_current_node = editor.currentNode()

    editor.setCurrentNode(node)
    apex_editor.setPwd(node.parent())
    apex_editor.setPin(hou.paneLinkType.Pinned)
    editor.setCurrentNode(old_current_node)

    
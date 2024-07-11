import hou

def connect(kwargs):
    pane = kwargs["pane"]
    # Check if the current pane is a Scene Viewer
    if isinstance(pane, hou.SceneViewer):
        scene_viewer = pane
        selection = scene_viewer.currentGeometrySelection()
        selection_strs = selection.selectionStrings()
        if selection_strs:
            selection_str = selection_strs[0]
            node = scene_viewer.currentNode()
            add_node = node.parent().createNode('add', node_name='connect_selected')

            # Add by group
            parm = add_node.parm("switcher1")
            parm.set(1)
            parm = add_node.parm("group")
            parm.set(selection_str)
            parm = add_node.parm("add")
            parm.set('all')

            add_node.setInput(0, node)
            add_node.moveToGoodPosition()

            # Select the new node
            scene_viewer.setCurrentNode(add_node)
            # Set display flags
            add_node.setDisplayFlag(True)
            if node.isRenderFlagSet():
                add_node.setRenderFlag(True)
            

            


        


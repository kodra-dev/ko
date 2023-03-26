import hou

def setViewportOrientation(kwargs, orientation):
    pane = kwargs["pane"]
    viewport = pane.curViewport()
    viewport.homeAll()

    newZ = hou.Vector3(0, 0, 1)
    newType = hou.geometryViewportType.Perspective
    if orientation == 'TOP':
        newType = hou.geometryViewportType.Top
    if orientation == 'BOTTOM':
        newType = hou.geometryViewportType.Bottom
    if orientation == 'RIGHT':
        newType = hou.geometryViewportType.Right
    if orientation == 'LEFT':
        newType = hou.geometryViewportType.Left
    if orientation == 'FRONT':
        newType = hou.geometryViewportType.Front
    if orientation == 'BACK':
        newType = hou.geometryViewportType.Back
    if orientation == "PERSPECTIVE":
        newType = hou.geometryViewportType.Perspective
    
    viewport.changeType(newType)
      
import hou

def setViewportOrientation(kwargs, orientation):
    pane = kwargs["pane"]
    viewport = pane.curViewport()
    viewport.homeAll()

    newZ = hou.Vector3(0, 0, 1)
    if orientation == 'TOP':
        newZ = hou.Vector3(0, -1, 0)
    if orientation == 'BOTTOM':
        newZ = hou.Vector3(0, 1, 0)
    if orientation == 'RIGHT':
        newZ = hou.Vector3(-1, 0, 0)
    if orientation == 'LEFT':
        newZ = hou.Vector3(1, 0, 0)
    if orientation == 'FRONT':
        newZ = hou.Vector3(0, 0, 1)
    if orientation == 'BACK':
        newZ = hou.Vector3(0, 0, -1)
      
    camera = viewport.defaultCamera()
    camera.setRotation(
        hou.hmath.buildRotateZToAxis(newZ).extractRotationMatrix3()
    )
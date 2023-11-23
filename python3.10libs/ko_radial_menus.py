import hou

def setViewportOrientation(kwargs, orientation, viewport):
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
    if orientation == "UV":
        newType = hou.geometryViewportType.UV
    
    viewport.changeType(newType)


def animationPlayback(kwargs, command):
    if command == "PLAY":
        if hou.playbar.isPlaying():
            hou.playbar.stop()
        else:
            hou.playbar.play()
    if command == "REVERSE":
        if hou.playbar.isPlaying():
            hou.playbar.stop()
        else:
            hou.playbar.reverse()
    if command == "NEXT_KEYFRAME":
        hou.playbar.stop()
        hou.playbar.jumpToNextKeyframe()
    if command == "PREV_KEYFRAME":
        hou.playbar.stop()
        hou.playbar.jumpToPreviousKeyframe()
    if command == "TO_LAST":
        range = hou.playbar.playbackRange()
        hou.playbar.stop()
        hou.setFrame(range[1])
    if command == "TO_FIRST":
        range = hou.playbar.playbackRange()
        hou.playbar.stop()
        hou.setFrame(range[0])
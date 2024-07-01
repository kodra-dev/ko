import hou
import nodegraphview

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


def SetQuickmarkMenu(editor):
    def create_quickmark_item(index):
        def wrapper(**kwargs):
            nodegraphview.createQuickMark(editor, index)
            editor.flashMessage(f"NETVIEW_quickmark_set_{index+1}", f"Quickmark {index+1} set", 1)
        
        qm = nodegraphview.getQuickMark(index)

        return {
            "type": "script_action",
            "label": nodegraphview.quickmarkMenuLabel(index).removeprefix("Quickmark "),
            "script": wrapper,
            "icon": "BUTTONS_quickmark" if qm is None else qm.currentnode.type().icon(),
        }

    menu = {
        "n": create_quickmark_item(0),
        "nw": create_quickmark_item(1),
        "w": create_quickmark_item(2),
        "sw": create_quickmark_item(3),
        "s": create_quickmark_item(4),
    }

    return menu


def GoQuickmarkMenu(editor):
    def create_quickmark_item(index):
        def wrapper(**kwargs):
            nodegraphview.jumpToQuickMark(editor, index)
            editor.flashMessage(f"NETVIEW_quickmark_go_{index+1}", f"Jump to Quickmark {index+1}", 1)

        qm = nodegraphview.getQuickMark(index)

        return {
            "type": "script_action",
            "label": nodegraphview.quickmarkMenuLabel(index).removeprefix("Quickmark "),
            "script": wrapper,
            "icon": "VIEW_display_point" if qm is None else qm.currentnode.type().icon(),
        }

    menu = {
        "n": create_quickmark_item(0),
        "nw": create_quickmark_item(1),
        "w": create_quickmark_item(2),
        "sw": create_quickmark_item(3),
        "s": create_quickmark_item(4),
    }

    return menu


def BookmarkMenu(editor):
    def icon_for(path):
        subs = path.split("/")
        if len(subs) == 2:
            if subs[1] == "obj":
                return 'NETWORKS_obj'
            if subs[1] == "chop":
                return 'NETWORKS_chop'
            if subs[1] == "img":
                return 'NETWORKS_cop2'
            if subs[1] == "shop":
                return 'NETWORKS_shop'
            if subs[1] == "mat":
                return 'NETWORKS_mat'
            if subs[1] == "tasks":
                return 'NETWORKS_top'
            if subs[1] == "out":
                return 'NETWORKS_rop'
            if subs[1] == "stage":
                return 'NETWORKS_lop'
        elif len(subs) > 2:
            node = hou.node(path)
            if node is not None:
                return node.type().icon()
        return 'VIEW_display_point'

    def create_wrapper(path):
        def wrapper(**kwargs):
            editor.cd(path)
        return wrapper

    bookmarks, err = hou.hscript("bookmark -l")
    if err:
        return {}
    bookmarks = bookmarks.splitlines()
    dirs = ["n", "ne", "e", "se", "s", "sw", "w"]
    menu = {
        dirs[i]: {
            "type": "script_action",
            "label": bookmarks[i],
            "script": create_wrapper(bookmarks[i]),
            "icon": icon_for(bookmarks[i]),
        } for i in range(min(len(bookmarks), 7))
    }

    return menu

    

import hou
import os

try:
    ko
except NameError:
    ko = {}

def bookmarkFilePath(node_name):
    hip = hou.text.expandString("$HIP")
    return os.path.join(hip, 'bookmarks', node_name + ".json")

def saveBookmarks(kwargs):
    node = kwargs['node']
    node_name = node.name()

    path = bookmarkFilePath(node_name)
    if not os.path.exists(os.path.dirname(path)):
        os.makedirs(os.path.dirname(path))

    success = hou.anim.saveBookmarks(path)
    if success:
        hou.ui.setStatusMessage(f"Saved to {path}")
    else:
        raise hou.Error("Failed to save bookmarks")

def loadBookmarks(kwargs):
    node = kwargs['node']
    node_name = node.name()

    success = hou.anim.loadBookmarks(bookmarkFilePath(node_name))
    if success:
        hou.ui.setStatusMessage(f"Loaded from {bookmarkFilePath(node_name)}")
    else:
        raise hou.Error("Failed to load bookmarks")
    

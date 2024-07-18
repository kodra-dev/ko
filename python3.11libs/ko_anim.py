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
    hou.anim.saveBookmarks(path)
    hou.ui.setStatusMessage(f"Saved to {path}")

def loadBookmarks(kwargs):
    node = kwargs['node']
    node_name = node.name()

    hou.anim.loadBookmarks(bookmarkFilePath(node_name))
    hou.ui.setStatusMessage(f"Loaded from {bookmarkFilePath(node_name)}")
    

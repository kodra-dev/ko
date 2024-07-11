import hou
import os

try:
    ko
except NameError:
    ko = {}

def bookmarkFilePath(node_name):
    hip = hou.text.expandString("$HIP")
    return os.path.join(hip, node_name + ".json")

def bookmarkFileExists(node_name):
    return os.path.isfile(bookmarkFilePath(node_name))

def saveBookmarks(node_name):
    path = bookmarkFilePath(node_name)
    hou.anim.saveBookmarks(path)
    hou.ui.setStatusMessage(f"Saved to {path}")

def loadBookmarks(node_name):
    hou.anim.loadBookmarks(bookmarkFilePath(node_name))
    ko['loaded_bookmarks_name'] = node_name


def saveLoadBookmarks(kwargs):
    node = kwargs['node']
    node_name = node.name()

    if 'loaded_bookmarks_name' in ko:
        loaded_name = ko['loaded_bookmarks_name']
        if loaded_name != node_name:
            confirm = hou.ui.displayConfirmation(f"Curren bookmarks are loaded from {loaded_name}. Save first?")
            if confirm:
                saveBookmarks(loaded_name)

            if bookmarkFileExists(node_name):
                loadBookmarks(node_name)
            else:
                saveBookmarks(node_name)
        else:
            saveBookmarks(node_name)
    else:
        if bookmarkFileExists(node_name):
            loadBookmarks(node_name)
        else:
            saveBookmarks(node_name)


    

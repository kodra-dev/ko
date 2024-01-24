import hou

with open(hou.text.expandString("$HFS/houdini/scripts/123.cmd")) as f:
    hou.hscript(f.read())
hou.setFps(30)
hou.playbar.setRealTime(True)

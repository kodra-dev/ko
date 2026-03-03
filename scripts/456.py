import hou

hou.hscript("autosave on")
hou.hscript("opunhide Sop shotsculpt")
hou.hscript("opunhide Sop elastictransform")

try:
    hou.setPreference('showmenu.val',['0','1'][1])
except hou.NotAvailable: # not available in houdini command line tools
    pass

import hou
import os
import sys
import subprocess
import importlib

def openInExplorer(show_dialog=False):
    if show_dialog:
        (_, path) = hou.ui.readInput("Opening:", buttons=("OK",), initial_contents="$HIP")
    else:
        path = "$HIP"

    os.startfile(hou.text.expandString(path))

def reloadKoModules():
    loaded_modules = list(sys.modules.keys())

    for mn in loaded_modules:
        if mn.startswith("ko_"):
            m = importlib.import_module(mn)
            importlib.reload(m)
        
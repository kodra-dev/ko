import hou
import os
import sys
import subprocess
import importlib

def openInExplorer():
    path = hou.getenv("HIP")
    os.startfile(path)

def reloadKoModules():
    loaded_modules = list(sys.modules.keys())

    for mn in loaded_modules:
        if mn.startswith("ko_"):
            m = importlib.import_module(mn)
            importlib.reload(m)
        
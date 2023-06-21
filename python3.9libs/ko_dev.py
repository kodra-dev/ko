import hou
import os
import subprocess

def openInExplorer():
    path = hou.getenv("HIP")
    os.startfile(path)
    
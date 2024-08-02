import hou
import os
import re

def clearCheckpoints(dop_node):
    file_pattern = dop_node.parm('explicitcachename').eval()
    file_pattern = file_pattern.replace("\\", "/")
    dir_path = os.path.dirname(file_pattern)
    if not os.path.exists(dir_path):
        hou.ui.setStatusMessage(f"{dir_path} does not exist", hou.severityType.Warning)
        return
    
    (pre, suf) = file_pattern.split("$SF")
    pattern = re.compile(re.escape(pre) + r'\d+' + re.escape(suf))
    
    checkpoints = [os.path.join(dir_path, f).replace("\\", "/") for f in os.listdir(dir_path)]
    checkpoints = [f for f in checkpoints if pattern.match(f)]
    counter = len(checkpoints)
    last_one = ""
    for checkpoint in checkpoints:
        os.remove(checkpoint)
        last_one = checkpoint

    hou.ui.setStatusMessage(f"Removed {counter} checkpoints. Last one: {last_one}", hou.severityType.Message)
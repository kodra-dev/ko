
def exportConstJoints(geo, fbx_filepath):
    if not geo.findGlobalAttrib("constjoints"):
        return

    filepath = fbx_filepath.replace(".fbx", "_constjoints.txt")
    with open(filepath, "w") as f:
        f.write(geo.attribValue("constjoints"))
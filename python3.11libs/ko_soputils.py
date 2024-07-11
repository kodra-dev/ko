def xordToString(xord):
    if xord == 0:
        return "srt"
    elif xord == 1:
        return "str"
    elif xord == 2:
        return "rst"
    elif xord == 3:
        return "rts"
    elif xord == 4:
        return "tsr"
    elif xord == 5:
        return "trs"
    else:
        raise ValueError(f"Invalid xord {xord}")

def rordToString(rord):
    if rord == 0:
        return "xyz"
    elif rord == 1:
        return "xzy"
    elif rord == 2:
        return "yxz"
    elif rord == 3:
        return "yzx"
    elif rord == 4:
        return "zxy"
    elif rord == 5:
        return "zyx"
    else:
        raise ValueError(f"Invalid rord {rord}")
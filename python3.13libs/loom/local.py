"""Read live Houdini settings. Never read/modify a keymap file or assume defaults."""
from __future__ import annotations

import platform

_cache = None


def settings():
    import hou
    result = {"platform": platform.system(), "houdini": hou.applicationVersionString(),
              "ui_available": hou.isUIAvailable(), "fps": hou.fps(),
              "update_mode": str(hou.updateModeSetting())}
    if hasattr(hou, "hotkeys"):
        result["keymap"] = hou.hotkeys.currentKeymap()
    else:
        result["keymap_unavailable"] = "Hotkey manager requires graphical Houdini"
    return result


def _normalized(text):
    return "".join(c for c in text.casefold() if c.isalnum())


def hotkeys(query, offset=0, limit=50):
    """Search the effective keymap, including context-specific/tool bindings.

    Cache only while Houdini's hotkey change index and keymap remain unchanged.
    No negative result may be presented as proof of a globally unbound shortcut.
    """
    import hou
    global _cache
    if not hasattr(hou, "hotkeys"):
        return {"available": False, "error": "Hotkey manager requires graphical Houdini"}
    manager = hou.hotkeys
    signature = (manager.currentKeymap(), manager.changeIndex())
    if _cache is None or _cache[0] != signature:
        records, pending, visited = [], ["h"], set()
        partial = False
        while pending:
            context = pending.pop()
            if context in visited:
                continue
            visited.add(context)
            if len(visited) > 10000 or len(records) > 100000:
                partial = True
                break
            pending.extend(child["symbol"] for child in manager.contextsInContext(context))
            for command in manager.commandBindingsInContext(context):
                symbol = command["symbol"]
                records.append({"context": context, "symbol": symbol,
                                "label": command.get("label", ""),
                                "description": command.get("help", ""),
                                "keys": list(manager.assignments(context, symbol))})
        records.sort(key=lambda record: (record["context"], record["symbol"]))
        _cache = (signature, records, partial)
    needle = _normalized(query)
    matches = [record for record in _cache[1]
               if needle in _normalized(" ".join((record["symbol"], record["label"],
                            record["description"], " ".join(record["keys"]))))]
    offset, limit = max(0, int(offset)), min(100, max(1, int(limit)))
    return {"available": True, "keymap": signature[0], "query": query,
            "source": "Live hou.hotkeys command bindings and resolved assignments",
            "matches": matches[offset:offset + limit], "total": len(matches),
            "partial": _cache[2], "next_offset": offset + limit if len(matches) > offset + limit else None,
            "note": "Bindings depend on context; search results do not establish which conflicting action wins. "
                    "No match is not proof that no custom scripted shortcut exists."}

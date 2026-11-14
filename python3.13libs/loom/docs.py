"""Read-only, root-confined access to local Houdini documentation."""
from pathlib import Path
import os
import re


def docs_root():
    configured = os.environ.get("LOOM_HOUDINI_DOCS")
    candidates = ([Path(configured)] if configured else [
        Path("H:/Docs/houdini_help"),
        Path(os.environ.get("HFS", "")) / "houdini" / "help"])
    for root in candidates:
        if root.is_dir():
            return root.resolve()
    raise ValueError("Local Houdini docs not found. Set LOOM_HOUDINI_DOCS to an extracted help directory.")


def inspect_docs(operation, path="/", query="", offset=0, limit=50, progress=lambda: None):
    root = docs_root()
    target = (root / path.lstrip("/")).resolve()
    if not target.is_relative_to(root):
        raise ValueError("Documentation path must stay inside the configured docs root")
    if operation == "docs_read":
        if not target.is_file() or target.suffix.lower() not in (".txt", ".md", ".html", ".htm"):
            raise ValueError("Choose a documentation text file returned by docs_search")
        with target.open(encoding="utf-8", errors="replace") as stream:
            lines = []
            more = False
            for number, line in enumerate(stream):
                if number < offset:
                    continue
                if len(lines) >= limit:
                    more = True
                    break
                lines.append(line[:2000].rstrip())
        return {"path": target.relative_to(root).as_posix(), "offset": offset,
                "text": "\n".join(lines), "next_offset": offset + limit if more else None}
    terms = re.findall(r"[\w:]+", query.lower())
    if not terms:
        raise ValueError("docs_search requires search words, e.g. subdivide or hou.NodeConnection")
    if not target.is_dir():
        raise ValueError("docs_search path must be a directory, or / for all docs")
    matches = []
    scanned = 0
    partial = False
    # Filename matches first, then bounded content search. Never execute doc examples.
    files = []
    for file in target.rglob("*"):
        if file.suffix.lower() in (".txt", ".md", ".html", ".htm") and file.is_file():
            if file.resolve().is_relative_to(root):
                files.append(file)
        if len(files) >= 50000:
            partial = True
            break
        if len(files) % 128 == 0:
            progress()
    files.sort(key=lambda p: (not all(t in p.as_posix().lower() for t in terms), p.as_posix()))
    for file in files:
        scanned += 1
        if scanned % 32 == 0:
            progress()
        relative = file.relative_to(root).as_posix()
        with file.open(encoding="utf-8", errors="replace") as stream:
            content = stream.read(65536)
        if all(t in relative.lower() for t in terms) or all(t in content.lower() for t in terms):
            line_number = next((i for i, line in enumerate(content.splitlines(), 1)
                                if any(t in line.lower() for t in terms)), 1)
            lines = content.splitlines()
            matches.append({"path": relative, "line": line_number,
                            "excerpt": "\n".join(lines[max(0, line_number - 2):line_number + 2])[:1200]})
        if len(matches) >= offset + limit + 1:
            partial = True
            break
    return {"root": str(root), "matches": matches[offset:offset + limit],
            "scanned": scanned, "partial": partial,
            "note": "Lexical search of paths and first 64 KiB per file. Narrow path/query for more results.",
            "next_offset": offset + limit if len(matches) > offset + limit else None}

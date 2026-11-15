"""Bounded text-file tools with exact edits and optimistic concurrency checks.

These checks guard tool operations, not arbitrary Python. Modes are not a sandbox.
"""
import hashlib
import json
import os
from pathlib import Path
import stat
import uuid

MAX_BYTES = 1_000_000
MAX_RESULT = 32_000
MUTATIONS = {"write", "edit"}


def requests_from_json(requests_json):
    if not isinstance(requests_json, str) or len(requests_json) > 2_000_000:
        raise ValueError("requests_json must be a string of at most 2 MB")
    requests = json.loads(requests_json)
    if not isinstance(requests, list) or not 1 <= len(requests) <= 20:
        raise ValueError("requests_json must contain 1–20 operations")
    allowed = {
        "read": {"offset", "limit"}, "list": {"offset", "limit"},
        "search": {"query", "offset", "limit"},
        "write": {"content", "expected_sha256"}, "edit": {"edits", "expected_sha256"},
    }
    for request in requests:
        if not isinstance(request, dict) or request.get("op") not in allowed:
            raise ValueError("Unknown file operation")
        if set(request) - ({"op", "path"} | allowed[request["op"]]):
            raise ValueError("Unexpected file operation fields")
        if not isinstance(request.get("path"), str) or not Path(request["path"]).is_absolute():
            raise ValueError("File paths must be absolute")
        for field in ("offset", "limit"):
            if field in request and (type(request[field]) is not int or request[field] < (0 if field == "offset" else 1)):
                raise ValueError("Invalid " + field)
    return requests


def _path(raw, write=False):
    path = Path(raw).resolve()
    parts = {part.lower() for part in path.parts}
    # Never read credentials or edit Git internals through this tool.
    if parts & {".git", ".ssh", ".aws", ".gnupg", ".codex"} or path.name.lower() in {
            "auth.json", "credentials", "credentials.json", ".env"}:
        raise PermissionError("Credential/Git paths are not available to Loom file tools")
    from .paths import data_root
    if path.is_relative_to((data_root() / "codex").resolve()):
        raise PermissionError("Provider state and credentials are not available to Loom file tools")
    if write:
        package = Path(__file__).resolve().parent
        repo = package.parent.parent
        if (path.is_relative_to(package) or path.is_relative_to(package.parent / "ko_ai")
                or (path.parent == repo / "python_panels" and path.name.startswith("loom"))
                or (path.parent == repo / "scripts" and path.name.startswith("loom"))
                or path == repo / "MainMenuCommon.xml"
                or path == repo / "toolbar" / "loom.shelf"):
            raise PermissionError("Do not edit Loom's running harness inside Houdini")
    return path


def _read(path):
    if path.stat().st_size > MAX_BYTES:
        raise ValueError("Text file exceeds 1 MB; use a smaller source file")
    data = path.read_bytes()
    if len(data) > MAX_BYTES or b"\0" in data:
        raise ValueError("Not a supported text file (binary or over 1 MB)")
    return data, data.decode("utf-8")


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _replace(path, content, expected):
    if not isinstance(content, str) or len(content.encode("utf-8")) > MAX_BYTES:
        raise ValueError("New text must be at most 1 MB")
    if not isinstance(expected, str):
        raise ValueError("expected_sha256 is required (empty only for a new file)")
    exists = path.exists()
    old = _read(path)[0] if exists else b""
    actual = _sha(old) if exists else ""
    if actual != expected:
        raise ValueError("File changed or already exists; read it again before editing")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        data = content.encode("utf-8")
        with temporary.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if exists:
            temporary.chmod(stat.S_IMODE(path.stat().st_mode))
        # Recheck before publication; this is not a filesystem transaction/lock.
        if (_sha(_read(path)[0]) if path.exists() else "") != expected:
            raise ValueError("File changed during the edit; nothing published")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    return {"path": str(path), "status": "written", "sha256": _sha(data),
            "bytes": len(data), "checkpoint_reversible": False}


def _operation(request, progress):
    op = request["op"]
    path = _path(request["path"], write=op in MUTATIONS)
    offset, limit = request.get("offset", 0), min(request.get("limit", 100), 200)
    if op == "read":
        data, content = _read(path)
        lines = content.splitlines(keepends=True)
        return {"path": str(path), "sha256": _sha(data), "offset": offset,
                "text": "".join(lines[offset:offset + limit]), "total_lines": len(lines),
                "next_offset": offset + limit if offset + limit < len(lines) else None}
    if op == "list":
        # Bounded traversal, not an unbounded recursive disk crawl.
        entries = []
        partial = False
        with os.scandir(path) as iterator:
            for item in iterator:
                progress()
                if len(entries) >= 2000:
                    partial = True
                    break
                try:
                    safe = _path(item.path)
                except PermissionError:
                    continue
                entries.append({"name": item.name, "directory": safe.is_dir()})
        entries.sort(key=lambda item: item["name"].lower())
        return {"path": str(path), "entries": entries[offset:offset + limit], "partial": partial,
                "next_offset": offset + limit if offset + limit < len(entries) else None}
    if op == "search":
        query = request.get("query")
        if not isinstance(query, str) or not query:
            raise ValueError("search requires a nonempty literal query")
        data, content = _read(path)
        matches = [{"line": i + 1, "text": line[:1000]} for i, line in enumerate(content.splitlines())
                   if query.casefold() in line.casefold()]
        return {"path": str(path), "sha256": _sha(data), "matches": matches[offset:offset + limit],
                "next_offset": offset + limit if offset + limit < len(matches) else None}
    if op == "write":
        return _replace(path, request.get("content"), request.get("expected_sha256"))
    if op == "edit":
        data, content = _read(path)
        if request.get("expected_sha256") != _sha(data):
            raise ValueError("File changed; read it again before editing")
        edits = request.get("edits")
        if not isinstance(edits, list) or not 1 <= len(edits) <= 100:
            raise ValueError("edit requires 1–100 exact replacements")
        spans = []
        for edit in edits:
            if (not isinstance(edit, dict) or set(edit) != {"old", "new"}
                    or not isinstance(edit["old"], str) or not edit["old"]
                    or not isinstance(edit["new"], str)):
                raise ValueError("Each edit requires unique nonempty old text and string new text")
            start = content.find(edit["old"])
            if start < 0 or content.find(edit["old"], start + 1) >= 0:
                raise ValueError("Old text must match exactly once in the original file")
            spans.append((start, start + len(edit["old"]), edit["new"]))
        spans.sort()
        if any(a[1] > b[0] for a, b in zip(spans, spans[1:])):
            raise ValueError("Edits must not overlap")
        for start, end, replacement in reversed(spans):
            content = content[:start] + replacement + content[end:]
        return _replace(path, content, request["expected_sha256"])
    raise ValueError("Unsupported operation")


def file_operations(requests, progress):
    results = []
    size = 0
    for request in requests:
        try:
            progress()
            result = _operation(request, progress)
        except Exception as exc:
            results.append({"path": request["path"], "error": str(exc)})
            return {"status": "failed", "results": results, "partial": len(results) > 1}
        encoded = json.dumps(result, ensure_ascii=False)
        if size + len(encoded) > MAX_RESULT:
            # Retain metadata; never drop a write result after performing it.
            for key in ("text", "matches", "entries"):
                if key in result:
                    result[key] = result[key][:max(0, MAX_RESULT - size - 1500)] if key == "text" else []
            result["truncated"] = True
            results.append(result)
            return {"status": "completed", "results": results, "remaining_requests_skipped": len(requests) - len(results)}
        results.append(result)
        size += len(encoded)
    return {"status": "completed", "results": results}

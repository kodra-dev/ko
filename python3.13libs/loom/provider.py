"""Official Codex app-server transport, using managed ChatGPT login only.

No token parsing/copying or unofficial ChatGPT endpoints. Credentials/config are
isolated in Loom's own CODEX_HOME; the user's ordinary Codex config is untouched.
"""
from __future__ import annotations

from collections import deque
import json
import os
from pathlib import Path
import queue
import shutil
import subprocess
import threading

from .checkpoints.storage import data_root


def find_codex():
    override = os.environ.get("LOOM_CODEX") or os.environ.get("KO_AI_CODEX")
    if override:
        path = Path(override)
        if not path.is_file():
            raise FileNotFoundError("LOOM_CODEX (or legacy KO_AI_CODEX) does not point to a file")
        return str(path)
    private = data_root() / "tools/node_modules/@openai"
    candidates = sorted(private.glob("codex-*/vendor/*/bin/codex.exe"))
    candidates += sorted(private.glob("codex-*/vendor/*/codex/codex.exe"))
    candidates += sorted(private.glob("codex-*/vendor/*/bin/codex"))
    candidates += sorted(private.glob("codex-*/vendor/*/codex/codex"))
    if candidates:
        return str(candidates[0])
    executable = shutil.which("codex")
    if executable and Path(executable).suffix.lower() not in (".cmd", ".bat", ".ps1"):
        return executable
    raise FileNotFoundError("Codex not found. Run scripts/loom_setup.ps1, then Connect again.")


def launch_config():
    home = data_root() / "codex"
    cwd = data_root() / "agent-workspace"
    home.mkdir(parents=True, exist_ok=True)
    cwd.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    # No API-key fallback, no inherited agent config, no inherited project directory.
    for key in ("OPENAI_API_KEY", "CODEX_API_KEY", "OPENAI_BASE_URL"):
        env.pop(key, None)
    env["CODEX_HOME"] = str(home)
    command = [find_codex(), "app-server", "--listen", "stdio://"]
    for setting in ("features.shell_tool=false", "features.unified_exec=false",
                    "features.code_mode=false", "features.code_mode_only=false",
                    # Dynamic Houdini tools require the host even with code-mode UI off.
                    "features.code_mode_host=true", "features.apply_patch_tool=false",
                    "features.apply_patch_freeform=false", "web_search=\"disabled\"",
                    "model_provider=\"openai\""):
        command += ["-c", setting]
    return command, env, cwd


class CodexTransport:
    def __init__(self):
        self.events = queue.Queue()
        self.stderr = deque(maxlen=40)
        self.process = None
        self._next_id = 0
        self._write_lock = threading.Lock()
        self._closing = False
        self._threads = []

    def start(self):
        if os.environ.get("LOOM_BLOCK_PROVIDER") == "1":
            raise RuntimeError("Provider access is disabled by the offline test runner. Paid tests require explicit approval.")
        if self.process and self.process.poll() is None:
            raise RuntimeError("Codex is already running")
        command, env, cwd = launch_config()
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True, encoding="utf-8",
                                        errors="replace", bufsize=1, env=env, cwd=str(cwd),
                                        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        self._threads = [threading.Thread(target=self._read_stdout, daemon=True),
                         threading.Thread(target=self._read_stderr, daemon=True)]
        for thread in self._threads:
            thread.start()

    def _read_stdout(self):
        try:
            for line in self.process.stdout:
                try:
                    self.events.put(json.loads(line))
                except json.JSONDecodeError:
                    self.events.put({"transport_error": "Invalid JSON from Codex app-server"})
        except Exception as exc:
            if not self._closing:
                self.events.put({"transport_error": str(exc)})
        finally:
            if not self._closing:
                self.events.put({"transport_error": "Codex app-server disconnected"})

    def _read_stderr(self):
        try:
            for line in self.process.stderr:
                self.stderr.append(line.rstrip())
        except (OSError, ValueError, AttributeError):
            if not self._closing:
                self.events.put({"transport_error": "Codex stderr pipe closed unexpectedly"})

    def send(self, message):
        if not self.process or self.process.poll() is not None:
            raise RuntimeError("Codex is not connected")
        with self._write_lock:
            self.process.stdin.write(json.dumps(message, ensure_ascii=False) + "\n")
            self.process.stdin.flush()

    def request(self, method, params=None):
        self._next_id += 1
        self.send({"id": self._next_id, "method": method, "params": params or {}})
        return self._next_id

    def notify(self, method, params=None):
        self.send({"method": method, "params": params or {}})

    def reply(self, request_id, result):
        self.send({"id": request_id, "result": result})

    def reject(self, request_id, message):
        self.send({"id": request_id, "error": {"code": -32601, "message": message}})

    def close(self):
        self._closing = True
        if self.process:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=3)
            for thread in self._threads:
                thread.join(timeout=1)
            for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
                if stream:
                    stream.close()
        self.process = None


def initialize_params():
    return {"clientInfo": {"name": "loom_houdini", "title": "Loom for Houdini", "version": "0.1.0"},
            "capabilities": {"experimentalApi": True}}


def resume_params(thread_id, model=None, instructions=None):
    """Use the relocated rollout explicitly; native Codex indexes may hold old paths."""
    import re
    result = thread_params(model, instructions)
    result["threadId"] = thread_id
    if re.fullmatch(r"[A-Za-z0-9_-]{1,128}", thread_id):
        sessions = data_root() / "codex" / "sessions"
        matches = list(sessions.rglob("*" + thread_id + ".jsonl"))
        if len(matches) == 1:
            result["path"] = str(matches[0])
    return result


def thread_params(model=None, instructions=None):
    from .tools import INSTRUCTIONS, tool_specs
    result = {"cwd": str((data_root() / "agent-workspace").resolve()),
              "modelProvider": "openai", "sandbox": "read-only", "approvalPolicy": "untrusted",
              "baseInstructions": INSTRUCTIONS, "dynamicTools": tool_specs()}
    if instructions is not None:
        result["developerInstructions"] = instructions
    if model:
        result["model"] = model
    return result

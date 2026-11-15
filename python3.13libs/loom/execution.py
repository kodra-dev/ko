"""In-process Python execution. Authorization is NOT a Python security sandbox."""
import contextlib
import io
import sys
import time
import traceback

MAX_CODE = 100_000
MAX_OUTPUT = 16_000


class ExecutionAborted(RuntimeError):
    """A failed progress/binding check, not an ordinary supplied-code error."""


class BoundedOutput(io.TextIOBase):
    def __init__(self):
        self.parts = []
        self.remaining = MAX_OUTPUT
        self.truncated = False

    def writable(self):
        return True

    def write(self, text):
        text = str(text)
        kept = text[:self.remaining]
        if kept:
            self.parts.append(kept)
        self.remaining -= len(kept)
        self.truncated |= len(text) > len(kept)
        return len(text)

    def value(self):
        return "".join(self.parts) + ("\n[output truncated]" if self.truncated else "")


def compile_python(code):
    if not isinstance(code, str) or not code.strip() or len(code) > MAX_CODE:
        raise ValueError("Python code must contain 1–100000 characters")
    return compile(code, "<loom-python>", "exec")


def run_python(compiled, progress):
    import hou
    if hou.isUIAvailable():
        from PySide6 import QtCore, QtWidgets
        app = QtWidgets.QApplication.instance()
        if app is None or QtCore.QThread.currentThread() != app.thread():
            raise RuntimeError("Houdini Python must execute on the UI thread")
    output = BoundedOutput()
    previous_trace = sys.gettrace()
    last_poll = time.monotonic()

    def check_progress():
        try:
            progress()
        except BaseException as exc:
            raise ExecutionAborted(str(exc)) from exc

    tick = check_progress

    def trace(frame, event, arg):
        nonlocal last_poll
        if event == "line" and time.monotonic() - last_poll >= 0.05:
            last_poll = time.monotonic()
            tick()
        return trace

    namespace = {"__name__": "__loom_execution__", "hou": hou,
                 "loom_check_cancelled": check_progress}
    error = None
    recoverable_error = False
    code_exception = None
    try:
        from .inspection import native_operation
        with native_operation("Loom: Python", check_progress) as tick:
            namespace["loom_check_cancelled"] = tick
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                sys.settrace(trace)
                try:
                    exec(compiled, namespace, namespace)
                except BaseException as exc:
                    code_exception = exc
                    raise
                finally:
                    sys.settrace(previous_trace)
    except BaseException as exc:
        # SystemExit/KeyboardInterrupt from supplied code must not kill Houdini.
        error = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))[-MAX_OUTPUT:]
        recoverable_error = (exc is code_exception and isinstance(exc, Exception)
                             and not isinstance(exc, (ExecutionAborted, hou.OperationInterrupted)))
    finally:
        sys.settrace(previous_trace)
    return {"output": output.value(), "error": error, "recoverable_error": recoverable_error,
            "warning": "UI, files and external effects are not restored by HIP checkpoints. "
                       "Cancellation is cooperative; native calls can block until they return."}

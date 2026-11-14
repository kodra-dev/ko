"""Loom's own quiet rising completion chime, not the system Asterisk used by pi.

No playback/import-time side effects. Windows playback is asynchronous; a missing
sound device must never turn a completed job into a harness error.
"""
import io
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import wave

from .checkpoints.storage import data_root


def completion_wave():
    rate = 22050
    frames = bytearray()
    # Three soft rising notes (C5/E5/A5), each with a click-free envelope.
    for frequency in (523.25, 659.25, 880.0):
        count = int(rate * .14)
        for sample in range(count):
            t = sample / rate
            envelope = min(1.0, t / .012) * max(0.0, 1 - sample / count) ** 1.5
            value = int(6500 * envelope * math.sin(2 * math.pi * frequency * t))
            frames.extend(struct.pack("<h", value))
        frames.extend(b"\0\0" * int(rate * .025))
    output = io.BytesIO()
    with wave.open(output, "wb") as sound:
        sound.setnchannels(1)
        sound.setsampwidth(2)
        sound.setframerate(rate)
        sound.writeframes(frames)
    return output.getvalue()


def completion_path():
    path = data_root() / "sounds" / "loom-complete-v1.wav"
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=path.parent, suffix=".wav", delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(completion_wave())
            os.replace(temporary, path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    return path


def play_completion():
    if sys.platform != "win32":
        return False
    try:
        import winsound
        winsound.PlaySound(str(completion_path()),
                           winsound.SND_FILENAME | winsound.SND_ASYNC | winsound.SND_NODEFAULT)
        return True
    except (ImportError, OSError, RuntimeError):
        return False

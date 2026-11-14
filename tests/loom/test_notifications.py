"""Completion chime tests never play real audio."""
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import wave

from loom import notifications


class NotificationTests(unittest.TestCase):
    def test_chime_is_short_non_silent_pcm_not_a_system_alias(self):
        data = notifications.completion_wave()
        with wave.open(io.BytesIO(data), "rb") as sound:
            self.assertEqual(sound.getnchannels(), 1)
            self.assertEqual(sound.getsampwidth(), 2)
            self.assertEqual(sound.getframerate(), 22050)
            duration = sound.getnframes() / sound.getframerate()
            self.assertGreater(duration, .4)
            self.assertLess(duration, .6)
            self.assertNotEqual(set(sound.readframes(sound.getnframes())), {0})

    def test_creates_and_reuses_local_sound_file(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(notifications, "data_root", return_value=Path(directory)):
                path = notifications.completion_path()
                self.assertEqual(path.read_bytes(), notifications.completion_wave())
                with patch.object(notifications, "completion_wave", side_effect=AssertionError("regenerated")):
                    self.assertEqual(notifications.completion_path(), path)
                self.assertEqual(list(path.parent.iterdir()), [path])

    def test_windows_playback_is_async_and_has_no_default_system_sound_fallback(self):
        sound = SimpleNamespace(PlaySound=Mock(), SND_FILENAME=1, SND_ASYNC=2, SND_NODEFAULT=4)
        with patch.object(notifications.sys, "platform", "win32"), patch.dict("sys.modules", winsound=sound):
            with patch.object(notifications, "completion_path", return_value=Path("loom-complete-v1.wav")):
                self.assertTrue(notifications.play_completion())
                sound.PlaySound.assert_called_once_with("loom-complete-v1.wav", 7)
                sound.PlaySound.side_effect = RuntimeError("no audio device")
                self.assertFalse(notifications.play_completion())
            with patch.object(notifications, "completion_path", side_effect=OSError("disk unavailable")):
                self.assertFalse(notifications.play_completion())

    def test_non_windows_does_not_play_another_sound(self):
        with patch.object(notifications.sys, "platform", "linux"), patch.object(notifications, "completion_path") as path:
            self.assertFalse(notifications.play_completion())
            path.assert_not_called()


if __name__ == "__main__":
    unittest.main()

"""Native bounded fork parameter checks; no transport/provider process."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from loom import provider


class ForkTests(unittest.TestCase):
    def test_before_and_after_use_native_boundaries_and_defer_auto_continuation(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(provider, "data_root", return_value=Path(directory)):
            for position, field in (("before", "beforeTurnId"), ("after", "lastTurnId")):
                params = provider.fork_params("source", "turn-1", position, "test-model", "guidance")
                self.assertEqual(params[field], "turn-1")
                self.assertEqual(params["threadId"], "source")
                self.assertEqual(params["model"], "test-model")
                self.assertEqual(params["developerInstructions"], "guidance")
                self.assertTrue(params["deferGoalContinuation"])
                self.assertTrue(params["excludeTurns"])
                self.assertNotIn("dynamicTools", params)
                self.assertEqual(params["sandbox"], "read-only")
                self.assertEqual(params["approvalPolicy"], "untrusted")
                self.assertNotIn("lastTurnId" if position == "before" else "beforeTurnId", params)

    def test_no_unbounded_fork_fallback(self):
        for turn, position in ((None, "before"), ("turn", "unknown")):
            with self.assertRaises(ValueError):
                provider.fork_params("source", turn, position)


if __name__ == "__main__":
    unittest.main()

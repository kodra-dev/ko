"""Offline token accounting; never starts a transport."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
from loom.usage import ThreadUsage, estimate


def report(inputs=100000, cached=40000, outputs=4000, writes=0, window=742857):
    counts = {"inputTokens": inputs, "cachedInputTokens": cached,
              "cacheWriteInputTokens": writes, "outputTokens": outputs,
              "reasoningOutputTokens": outputs // 2, "totalTokens": inputs + outputs}
    return {"total": counts, "last": dict(counts), "modelContextWindow": window}


class UsageTests(unittest.TestCase):
    def test_cached_input_and_reasoning_not_counted_twice(self):
        usage = ThreadUsage()
        usage.update(report(), "gpt-5")
        self.assertAlmostEqual(usage.state["cost"], .12)
        self.assertEqual(usage.label(), "≈$0.12 C14%")

    def test_duplicate_notifications_and_model_switch(self):
        usage = ThreadUsage()
        usage.update(report(), "gpt-5")
        usage.update(report(), "gpt-5.4")
        self.assertAlmostEqual(usage.state["cost"], .12)
        next_report = report(200000, 80000, 8000)
        next_report["last"] = report()["last"]
        usage.update(next_report, "gpt-5.4")
        self.assertAlmostEqual(usage.state["cost"], .12 + .15 + .01 + .06)

    def test_compaction_changes_context_not_cumulative_cost(self):
        usage = ThreadUsage()
        data = report()
        usage.update(data, "gpt-5")
        data["last"]["totalTokens"] = 20000
        data["modelContextWindow"] = 100000
        usage.update(data, "gpt-5")
        self.assertEqual(usage.label(), "≈$0.12 C20%")

    def test_cache_writes_and_long_context_rates(self):
        counts = report(300000, 100000, 1000, 50000)["total"]
        expected = (150000 * 4 + 100000 * .2 + 50000 * 5 + 1000 * 15) / 1e6
        self.assertAlmostEqual(estimate("gpt-6.1-sol", counts, 300000), expected)
        self.assertIsNone(estimate("gpt-5", counts, 300000))

    def test_unknown_model_missing_window_and_untracked_history(self):
        usage = ThreadUsage()
        usage.update(report(window=None), "future-model")
        self.assertEqual(usage.label(), "≈$— C—%")
        with tempfile.TemporaryDirectory() as directory:
            usage = ThreadUsage.load(Path(directory), "old-thread")
            usage.update(report(), "gpt-5")
            self.assertEqual(usage.label(), "≈$— C14%")

    def test_persistence_and_reset(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            usage = ThreadUsage()
            usage.update(report(), "gpt-5")
            usage.save(root, "thread/unsafe-name")
            restored = ThreadUsage.load(root, "thread/unsafe-name")
            restored.update(report(), "gpt-5.4")
            self.assertEqual(restored.label(), "≈$0.12 C14%")
            self.assertEqual(ThreadUsage.load(root, None).label(), "≈$0.00 C—%")
            self.assertEqual(ThreadUsage.load(root, "other-thread").label(), "≈$— C—%")

    def test_invalid_events_and_counter_resets_do_not_make_negative_cost(self):
        usage = ThreadUsage()
        usage.update({}, "gpt-5")
        self.assertEqual(usage.label(), "≈$0.00 C—%")
        usage.update(report(), "gpt-5")
        usage.update(report(1000, 0, 100), "gpt-5")
        self.assertEqual(usage.label().split()[0], "≈$—")
        self.assertAlmostEqual(usage.state["cost"], .12)

    def test_unknown_names_are_not_guessed_from_prefix(self):
        counts = report()["total"]
        self.assertIsNone(estimate("gpt-5-super-fast", counts, 100000))
        self.assertEqual(estimate("gpt-5-2025-08-07", counts, 100000), .12)


if __name__ == "__main__":
    unittest.main()

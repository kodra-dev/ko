"""Regression checks for focused offline selection and timing (ordinary Python)."""
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import loom_test_runner as runner


class RunnerTests(unittest.TestCase):
    def test_fast_modules_are_present_and_do_not_need_houdini_or_qt(self):
        # The Fast suite itself exercises this under ordinary Python.
        for module in runner.FAST_MODULES:
            self.assertTrue((runner.TESTS / (module + ".py")).is_file())

    def test_module_class_and_method_selection(self):
        module = runner.load_suite("fast", ["test_usage"])
        cls = runner.load_suite("fast", ["test_usage.UsageTests"])
        method = runner.load_suite("fast", ["test_usage.UsageTests.test_persistence_and_reset"])
        self.assertEqual(module.countTestCases(), cls.countTestCases())
        self.assertGreater(module.countTestCases(), 1)
        self.assertEqual(method.countTestCases(), 1)

    def test_rejects_paid_gui_paths_and_unknown_modules(self):
        for name in ("paid.provider_roundtrip", "gui_startup", "../paid", "test_missing"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                runner.load_suite("unit", [name])
        with self.assertRaises(ValueError):
            runner.load_suite("fast", ["test_live_settings"])

    def test_unknown_method_is_an_error_not_an_empty_success(self):
        with self.assertRaises(ValueError):
            runner.load_suite("fast", ["test_usage.UsageTests.test_missing"])

    def test_empty_selection_fails(self):
        with patch.object(runner, "FAST_MODULES", ()):
            with self.assertRaisesRegex(ValueError, "No tests"):
                runner.load_suite("fast")

    def test_integration_selector_accepts_short_class_names(self):
        loader = unittest.TestLoader()
        with patch.object(unittest, "TestLoader", return_value=loader), \
                patch.object(loader, "loadTestsFromNames", return_value=unittest.TestSuite([unittest.FunctionTestCase(lambda: None)])) as load:
            runner.load_suite("ui", ["UiTests.test_example"])
            load.assert_called_once_with(["ui_smoke.UiTests.test_example"])

    def test_timing_preserves_failure_and_skip_results(self):
        class Examples(unittest.TestCase):
            def test_pass(self):
                pass

            def test_fail(self):
                self.fail("expected regression fixture")

            @unittest.skip("fixture")
            def test_skip(self):
                pass

        result = unittest.TextTestRunner(stream=io.StringIO(), resultclass=runner.TimedResult).run(
            unittest.defaultTestLoader.loadTestsFromTestCase(Examples))
        self.assertFalse(result.wasSuccessful())
        self.assertEqual(len(result.failures), 1)
        self.assertEqual(len(result.skipped), 1)
        self.assertEqual(len(result.timings), 3)
        self.assertTrue(all(seconds >= 0 for seconds, _ in result.timings))

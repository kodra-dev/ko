"""Offline unittest entry point; each invocation owns a fresh interpreter and home."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

REPO = Path(__file__).resolve().parents[1]
TESTS = REPO / "tests" / "loom"
# Deliberately explicit: adding a HOM/Qt test must not silently slow the fast lane.
FAST_MODULES = (
    "test_branch_provider", "test_conversations", "test_history",
    "test_notifications", "test_panel_navigation", "test_usage", "test_test_runner",
)
MODULES = {"houdini": "houdini_smoke", "ui": "ui_smoke", "reload": "ui_package_reload"}


class TimedResult(unittest.TextTestResult):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.timings = []

    def startTest(self, test):
        self._started = time.perf_counter()
        super().startTest(test)

    def stopTest(self, test):
        self.timings.append((time.perf_counter() - self._started, test.id()))
        super().stopTest(test)


def load_suite(name, selectors=()):
    """Only top-level offline modules are selectable, never paid/GUI entry points."""
    loader = unittest.TestLoader()
    if name == "fast":
        allowed = set(FAST_MODULES)
    elif name == "unit":
        allowed = {p.stem for p in TESTS.glob("test_*.py")}
    else:
        allowed = {MODULES[name]}
    if selectors:
        names = []
        for selector in selectors:
            if name in MODULES and selector.split(".")[0] not in allowed:
                selector = MODULES[name] + "." + selector
            if selector.split(".")[0] not in allowed:
                raise ValueError(f"Not an offline {name} test module: {selector}")
            names.append(selector)
        suite = loader.loadTestsFromNames(names)
    elif name == "unit":
        suite = loader.discover(str(TESTS), pattern="test_*.py")
    else:
        suite = loader.loadTestsFromNames(sorted(allowed))
    if loader.errors:
        raise ValueError("\n".join(loader.errors))
    if suite.countTestCases() == 0:
        raise ValueError("No tests selected")
    return suite


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=("fast", "unit", "houdini", "ui", "reload"), required=True)
    parser.add_argument("--test", action="append", default=[], help="Dotted module, class, or method (repeatable)")
    args = parser.parse_args(argv)
    started = time.perf_counter()
    # Set before loading any tests; subprocesses inherit these offline safeguards.
    os.environ["LOOM_BLOCK_PROVIDER"] = "1"
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    sys.path[:0] = [str(REPO / "python3.13libs"), str(TESTS)]
    with tempfile.TemporaryDirectory(prefix="loom-tests-") as home:
        os.environ["LOOM_HOME"] = home
        os.environ.pop("KO_AI_HOME", None)
        suite = load_suite(args.suite, args.test)
        loaded = time.perf_counter()
        print(f"{args.suite}: {suite.countTestCases()} tests; import/discovery {loaded - started:.3f}s", flush=True)
        runner = unittest.TextTestRunner(verbosity=2, resultclass=TimedResult)
        result = runner.run(suite)
        print("Slowest tests (includes setup/teardown):", flush=True)
        for seconds, test_id in sorted(result.timings, reverse=True)[:10]:
            print(f"  {seconds:8.3f}s  {test_id}", flush=True)
        print(f"{args.suite}: execution {time.perf_counter() - loaded:.3f}s", flush=True)
        return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())

"""Offline guards for network-independent Python Panels."""
from pathlib import Path
import sys
from types import ModuleType
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET


PANELS = Path(__file__).resolve().parents[2] / "python_panels"


class PanelNavigationTests(unittest.TestCase):
    def test_all_definitions_disable_navigation(self):
        for filename in ("loom.pypanel", "loom_checkpoints.pypanel", "loom_legacy.pypanel"):
            with self.subTest(filename=filename):
                interface = ET.parse(PANELS / filename).find("interface")
                self.assertEqual(interface.get("showNetworkNavigationBar"), "false")

    def test_reload_hides_controls_on_already_installed_panes(self):
        # Stop at registration, before any Qt widgets or controllers are created.
        class Registered(Exception):
            pass

        for filename in ("loom.pypanel", "loom_checkpoints.pypanel"):
            with self.subTest(filename=filename):
                pane = Mock()
                remember = Mock(side_effect=Registered)
                loom = ModuleType("loom")
                loom._remember_pane = remember
                checkpoints = ModuleType("loom.checkpoints")
                checkpoints._remember_pane = remember
                checkpoints.create_panel = Mock()
                modules = {"hou": ModuleType("hou"), "loom": loom,
                           "loom.checkpoints": checkpoints}
                namespace = {"kwargs": {"paneTab": pane}}
                script = ET.parse(PANELS / filename).find("interface/script").text
                with patch.dict(sys.modules, modules):
                    exec(compile(script, filename, "exec"), namespace)
                    with self.assertRaises(Registered):
                        namespace["onCreateInterface"]()
                pane.showNetworkControls.assert_called_once_with(False)
                remember.assert_called_once_with(pane)


if __name__ == "__main__":
    unittest.main()

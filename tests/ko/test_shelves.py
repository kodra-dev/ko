"""Portable shelf-set checks; no artist preferences or Houdini process needed."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
EXPECTED = ["ko_dev", "ko_modeling", "ko_network", "ko_rigging", "ko_anim"]


class ShelfTests(unittest.TestCase):
    def test_kodra_contains_all_personal_shelves(self):
        root = ET.parse(ROOT / "toolbar/kodra.shelf").getroot()
        shelf_set = root.find("shelfSet[@name='kodra']")
        self.assertIsNotNone(shelf_set)
        self.assertEqual([e.get("name") for e in shelf_set.findall("memberToolshelf")], EXPECTED)

    def test_referenced_shelves_are_defined_in_package(self):
        definitions = {}
        for path in (ROOT / "toolbar").glob("*.shelf"):
            for shelf in ET.parse(path).getroot().findall("toolshelf"):
                self.assertNotIn(shelf.get("name"), definitions, "Duplicate shelf definition")
                definitions[shelf.get("name")] = path
        for name in EXPECTED:
            self.assertIn(name, definitions)

    def test_migrated_tool_definitions_do_not_depend_on_default_shelf(self):
        required = {
            "ko_modeling.shelf": ["ko_edit", "ko_edit_mirror"],
            "ko_rigging.shelf": ["kodra::control_shapes_post_rigging", "unpack_edit_repack_rig"],
        }
        for filename, names in required.items():
            root = ET.parse(ROOT / "toolbar" / filename).getroot()
            for name in names:
                tools = [e for e in root.findall("tool") if e.get("name") == name]
                self.assertEqual(len(tools), 1, f"{filename}: missing or duplicate {name}")
                self.assertTrue((tools[0].findtext("script") or "").strip())


if __name__ == "__main__":
    unittest.main(verbosity=2)

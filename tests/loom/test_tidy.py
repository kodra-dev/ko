"""Offline scope and structured cleanup operations, using disposable Houdini nodes."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
import hou
from loom.tidy import resolve_scope, TidyScope, request_text
from loom.tools import SceneTools, validate_steps


class TidyTests(unittest.TestCase):
    def setUp(self):
        self.parent = hou.node('/obj').createNode('geo', 'tidy_test')
        self.a = self.parent.createNode('null', 'long_source_name')
        self.b = self.parent.createNode('null', 'result')
        self.c = self.parent.createNode('null', 'unrelated')
        self.b.setInput(0, self.a)
        self.api = SimpleNamespace(selectedNodes=lambda: [self.a])
        self.tools = object.__new__(SceneTools)
        self.tools.tidy_scope = TidyScope(self.parent, {self.a, self.b})

    def tearDown(self):
        self.parent.destroy()

    def test_selection_rules(self):
        self.assertEqual(resolve_scope(self.api)[1], {self.a, self.b})
        self.api.selectedNodes = lambda: [self.a, self.c]
        self.assertEqual(resolve_scope(self.api)[1], {self.a, self.c})
        self.api.selectedNodes = lambda: []
        editor = SimpleNamespace(type=lambda: 'network', isCurrentTab=lambda: True, pwd=lambda: self.parent)
        self.api.ui = SimpleNamespace(paneTabs=lambda: [editor])
        self.api.paneTabType = SimpleNamespace(NetworkEditor='network')
        self.assertEqual(resolve_scope(self.api)[1], {self.a, self.b, self.c})
        self.assertIn('behavior-preserving', request_text(self.parent, {self.a}))

    def test_scope_blocks_unselected_edits_and_full_layout(self):
        for step in ({'op': 'delete', 'path': self.c.path()},
                     {'op': 'layout', 'path': self.parent.path(), 'all': True},
                     {'op': 'layout', 'path': self.parent.path(), 'nodes': [self.a.path()]},
                     {'op': 'position', 'path': self.c.path(), 'value': [0, 0]}):
            with self.assertRaises(PermissionError):
                self.tools._apply_step(step)

    def test_position_validation_and_layout_guidance(self):
        for value in ([1], [True, 2], [float('nan'), 0], [0, float('inf')], '0,1'):
            with self.assertRaises(ValueError):
                validate_steps([dict(op='position', path=self.a.path(), value=value)])
        text = request_text(self.parent, {self.a})
        for requirement in ('2.25', 'rounded', 'tall', 'semantic roles', 'stale or incorrect comments',
                            'do not rely on Houdini auto-layout'):
            self.assertIn(requirement, text)

    def test_rename_layout_and_annotations(self):
        old_position = self.c.position()
        steps = [dict(op='rename', path=self.a.path(), name='source')]
        validate_steps(steps)
        self.tools._apply_step(steps[0])
        steps = [dict(op='position', path=self.a.path(), value=[0, 2]),
                 dict(op='position', path=self.b.path(), value=[0, 0]),
                 dict(op='comment', path=self.b.path(), text='Important boundary'),
                 dict(op='sticky', parent=self.parent.path(), text='Difficult section'),
                 dict(op='box', parent=self.parent.path(), text='Processing', nodes=[self.a.path(), self.b.path()])]
        validate_steps(steps)
        for step in steps:
            self.tools._apply_step(step)
        self.assertEqual(self.c.position(), old_position)
        self.assertEqual(self.a.name(), 'source')
        self.assertEqual(tuple(self.a.position()), (0, 2))
        self.assertEqual(tuple(self.b.position()), (0, 0))
        records = self.tools.inspect('overview', self.parent.path())['nodes']
        record = next(item for item in records if item['path'] == self.b.path())
        self.assertEqual(record['comment'], 'Important boundary')
        self.assertEqual(record['position'], [0, 0])
        self.tools._apply_step(dict(op='comment', path=self.b.path(), text='Corrected explanation'))
        self.assertEqual(self.b.comment(), 'Corrected explanation')
        self.tools._apply_step(dict(op='comment', path=self.b.path(), text=''))
        self.assertEqual(self.b.comment(), '')
        self.assertEqual(len(self.parent.stickyNotes()), 1)
        self.assertEqual(len(self.parent.networkBoxes()), 1)
        self.assertEqual(self.b.inputs()[0], self.a)


if __name__ == '__main__':
    unittest.main()

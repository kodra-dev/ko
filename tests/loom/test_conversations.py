"""Offline persistence/migration tests; no provider or Houdini required."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from loom.conversations import Conversations
from loom.checkpoints.storage import atomic_json, read_json


class ConversationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_migrates_all_legacy_histories_as_ordinary_conversations_once(self):
        legacy = {"messages": [{"role": "Tool", "text": "result", "success": True}],
                  "thread_id": "native-old", "context": {"retained": True}}
        atomic_json(self.root / "chat.json", legacy)
        atomic_json(self.root / "conversations" / "old.json", {"messages": [], "thread_id": "archived"})
        store = Conversations(self.root)
        self.assertEqual(store.current["messages"], legacy["messages"])
        self.assertEqual(store.current["thread_id"], "native-old")
        self.assertEqual(store.current["context"], legacy["context"])
        self.assertTrue(store.current["legacy_proposals"])
        self.assertEqual(len(store.records), 2)
        self.assertNotIn("archived", store.records[1])
        store.select(store.records[1]["id"])
        self.assertEqual(store.current["thread_id"], "archived")
        self.assertEqual(read_json(self.root / "chat.json"), legacy)
        store.create()
        self.assertEqual(len(Conversations(self.root).records), 3)

    def test_switch_reload_rename_preserves_history_and_selection(self):
        store = Conversations(self.root)
        first = store.current["id"]
        store.update(messages=[{"role": "You", "text": "first"}], thread_id="native", model="model-a", draft="draft")
        second = store.create("Second", model="model-b")
        store.rename(first, "First")
        store.select(first)
        store = Conversations(self.root)
        self.assertEqual(store.current["name"], "First")
        self.assertEqual(store.current["draft"], "draft")
        self.assertEqual(store.current["model"], "model-a")
        self.assertEqual(store.current["thread_id"], "native")
        store.select(second)
        self.assertEqual(store.current["id"], second)
        store.select(first)
        self.assertEqual(store.current["messages"][0]["text"], "first")

    def test_failed_publication_does_not_change_memory_or_disk(self):
        store = Conversations(self.root)
        before = read_json(store.path)
        with patch("loom.conversations.atomic_json", side_effect=OSError("disk full")):
            for action in (lambda: store.create(), lambda: store.delete(store.current["id"]),
                           lambda: store.rename(store.current["id"], "new"),
                           lambda: store.update(messages=[{"text": "lost"}])):
                with self.assertRaises(OSError):
                    action()
                self.assertEqual(store.state, before)
                self.assertEqual(read_json(store.path), before)

    def test_delete_selected_and_last_record_does_not_resurrect_on_reopen(self):
        atomic_json(self.root / "chat.json", {"messages": [{"role": "You", "text": "legacy"}]})
        store = Conversations(self.root)
        first = store.current["id"]
        second = store.create()
        store.delete(first)
        self.assertEqual(store.current["id"], second)
        store.delete(second)
        self.assertNotIn(store.current["id"], (first, second))
        self.assertEqual(len(Conversations(self.root).records), 1)
        self.assertEqual(store.current["messages"], [])
        before = store.records
        with patch("loom.conversations.atomic_json", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                store.delete(store.current["id"])
        self.assertEqual(store.records, before)

    def test_ai_name_is_short_one_time_and_never_overwrites_manual_name(self):
        store = Conversations(self.root)
        key = store.current["id"]
        self.assertTrue(store.rename(key, "  Cube\n  modeling  ", generated=True))
        self.assertEqual(store.current["name"], "Cube modeling")
        self.assertEqual(store.current["name_source"], "ai")
        self.assertFalse(store.rename(key, "Different", generated=True))
        key = store.create()
        store.rename(key, "My name")
        self.assertFalse(store.rename(key, "AI name", generated=True))
        self.assertEqual(Conversations(self.root).current["name"], "My name")

    def test_existing_hidden_conversations_become_normal_without_losing_selection(self):
        store = Conversations(self.root)
        first = store.current["id"]
        second = store.create("Previously hidden")
        store.update(messages=[{"role": "You", "text": "preserved"}], thread_id="native-hidden")
        state = read_json(store.path)
        state["records"][0]["archived"] = False
        state["records"][1]["archived"] = True
        atomic_json(store.path, state)
        store = Conversations(self.root)
        self.assertEqual(store.current["id"], second)
        self.assertEqual(store.current["thread_id"], "native-hidden")
        self.assertTrue(all("archived" not in r for r in read_json(store.path)["records"]))
        store.select(first)
        store.select(second)
        self.assertEqual(store.current["messages"][0]["text"], "preserved")
        self.assertFalse(hasattr(store, "archive"))

    def _branch_history(self):
        store = Conversations(self.root)
        store.update(thread_id="native-source", model="model-a", draft="original draft", messages=[
            {"role": "You", "text": "First", "turn_id": "t1", "branch_point": "before"},
            {"role": "Loom", "text": "Partial", "turn_id": "t1"},
            {"role": "Tool", "text": "result", "turn_id": "t1"},
            {"role": "Tool", "text": "tail result", "turn_id": "t1"},
            {"role": "Loom", "text": "Final", "turn_id": "t1", "branch_point": "after"},
            {"role": "You", "text": "Future secret", "turn_id": "t2", "branch_point": "before"}])
        return store

    def test_branch_before_prompt_excludes_entire_turn_and_preserves_original(self):
        store = self._branch_history()
        original = store.current
        store.branch(0, "native-fork")
        self.assertEqual(store.current["messages"], [])
        self.assertEqual(store.current["draft"], "First")
        self.assertEqual(store.current["model"], "model-a")
        self.assertEqual(store.current["thread_id"], "native-fork")
        self.assertEqual(store.current["branched_from"]["position"], "before")
        self.assertEqual(store.records[0], original)
        self.assertEqual(Conversations(self.root).current, store.current)

    def test_branch_after_reply_includes_whole_turn_but_not_future(self):
        store = self._branch_history()
        store.branch(4, "native-fork")
        self.assertEqual(len(store.current["messages"]), 5)
        self.assertEqual(store.current["messages"][-1]["text"], "Final")
        self.assertNotIn("Future secret", str(store.current))
        self.assertEqual(store.current["draft"], "")
        self.assertNotIn("legacy_proposals", store.current)

    def test_branch_rejects_unrecorded_intermediate_and_reused_native_thread(self):
        store = self._branch_history()
        before = store.records
        for index, thread in ((1, "fork"), (2, "fork"), (99, "fork"), (True, "fork"),
                              (0, "native-source"), (0, None)):
            with self.assertRaises(ValueError):
                store.branch(index, thread)
        self.assertEqual(store.records, before)
        with patch("loom.conversations.atomic_json", side_effect=OSError("full")):
            with self.assertRaises(OSError):
                store.branch(0, "fork")
        self.assertEqual(store.records, before)

    def test_replay_branch_works_at_exact_intermediate_message_without_future(self):
        store = self._branch_history()
        original = store.current
        store.branch(1, replay=True)
        self.assertEqual([m["text"] for m in store.current["messages"]], ["First", "Partial"])
        self.assertIsNone(store.current["thread_id"])
        self.assertEqual(store.current["replay_count"], 2)
        self.assertTrue(store.current["replay_pending"])
        self.assertTrue(all("turn_id" not in m for m in store.current["messages"]))
        self.assertEqual(store.records[0], original)
        self.assertEqual(Conversations(self.root).current, store.current)

    def test_trim_keeps_conversation_name_and_draft_but_resets_provider_and_proposal_scope(self):
        store = self._branch_history()
        old = store.current
        store.trim(3)
        now = store.current
        self.assertEqual(now["id"], old["id"])
        self.assertEqual(now["name"], old["name"])
        self.assertEqual(now["draft"], old["draft"])
        self.assertEqual(len(store.records), 1)
        self.assertEqual([m["text"] for m in now["messages"]], ["First", "Partial", "result"])
        self.assertIsNone(now["thread_id"])
        self.assertTrue(now["replay_pending"])
        self.assertTrue(now["context_epoch"])
        self.assertTrue(all("turn_id" not in m for m in now["messages"]))
        self.assertNotIn("Future secret", str(now))
        self.assertEqual(Conversations(self.root).current, now)

    def test_edit_prompt_refills_draft_and_delete_first_prompt_clears_history(self):
        store = self._branch_history()
        store.trim(5, edit=True)
        self.assertEqual(store.current["draft"], "Future secret")
        self.assertEqual(len(store.current["messages"]), 5)
        store.trim(0)
        self.assertEqual(store.current["messages"], [])
        self.assertFalse(store.current["replay_pending"])
        self.assertEqual(store.current["draft"], "Future secret")

    def test_trim_failure_does_not_mutate_history_and_edit_rejects_assistant(self):
        store = self._branch_history()
        original = store.current
        with self.assertRaises(ValueError):
            store.trim(1, edit=True)
        with patch("loom.conversations.atomic_json", side_effect=OSError("full")):
            with self.assertRaises(OSError):
                store.trim(0)
        self.assertEqual(store.current, original)

    def test_before_first_replayed_turn_must_keep_replayed_history(self):
        store = self._branch_history()
        store.update(replay_turn_id="t2")
        self.assertFalse(store.branch_point(5)["native"])
        store.branch(5, replay=True)
        self.assertEqual(len(store.current["messages"]), 5)
        self.assertTrue(store.current["replay_pending"])

    def test_invalid_store_is_not_overwritten(self):
        atomic_json(self.root / "conversations.json", {"version": 99})
        with self.assertRaises(ValueError):
            Conversations(self.root)
        self.assertEqual(read_json(self.root / "conversations.json"), {"version": 99})


if __name__ == "__main__":
    unittest.main()

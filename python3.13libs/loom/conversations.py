"""Workspace-local conversations. No Houdini, Qt, or provider dependency.

One atomic publication keeps selection and records consistent. Legacy history
files remain untouched; an existing store is authoritative on subsequent opens.
The workspace service owns the process lock (this is not a multi-writer database).
"""
from copy import deepcopy
import uuid

from .checkpoints.storage import atomic_json, read_json


class Conversations:
    def __init__(self, root):
        self.path = root / "conversations.json"
        if self.path.exists():
            self.state = read_json(self.path)
            if self.state.get("version") != 1:
                raise ValueError("Unsupported conversation store version")
            self.current  # Fail closed rather than replace damaged history.
            migrated = any("archived" in r for r in self.state["records"])
            for record in self.state["records"]:
                record.pop("archived", None)  # Old hidden conversations become ordinary items.
                record.setdefault("name_source", "pending" if not record["messages"]
                                  and record["name"] == "Conversation" else "existing")
            if migrated:
                atomic_json(self.path, self.state)
            return
        legacy = root / "chat.json"
        record = self._record("Conversation", read_json(legacy) if legacy.exists() else {})
        record["legacy_proposals"] = True
        records = [record]
        for path in sorted((root / "conversations").glob("*.json")):
            records.append(self._record("Imported " + path.stem[:8], read_json(path)))
        self.state = {"version": 1, "selected": record["id"], "records": records}
        atomic_json(self.path, self.state)

    @staticmethod
    def _record(name, saved):
        saved = {key: value for key, value in saved.items() if key != "archived"}
        return {**saved, "id": uuid.uuid4().hex, "name": name,
                "messages": saved.get("messages", []), "thread_id": saved.get("thread_id"),
                "model": saved.get("model"), "draft": saved.get("draft", ""),
                "name_source": saved.get("name_source", "existing" if saved.get("messages") else "pending")}

    @property
    def records(self):
        return deepcopy(self.state["records"])

    @property
    def current(self):
        record = self._find(self.state, self.state["selected"])
        return deepcopy(record)

    @staticmethod
    def _find(state, key):
        for record in state["records"]:
            if record["id"] == key:
                return record
        raise ValueError("Conversation not found")

    def _publish(self, state):
        atomic_json(self.path, state)
        self.state = state

    def update(self, **fields):
        if set(fields) - {"messages", "thread_id", "model", "draft", "replay_pending", "replay_turn_id"}:
            raise ValueError("Unsupported conversation field")
        state = deepcopy(self.state)
        self._find(state, state["selected"]).update(deepcopy(fields))
        self._publish(state)

    def create(self, name="Conversation", model=None):
        state = deepcopy(self.state)
        record = self._record(name.strip() or "Conversation", {"model": model})
        state["records"].append(record)
        state["selected"] = record["id"]
        self._publish(state)
        return record["id"]

    def select(self, key):
        state = deepcopy(self.state)
        self._find(state, key)
        state["selected"] = key
        self._publish(state)

    def rename(self, key, name, generated=False):
        name = name.strip()
        if not name:
            raise ValueError("Conversation name cannot be empty")
        state = deepcopy(self.state)
        record = self._find(state, key)
        if generated:
            if record.get("name_source") != "pending":
                return False  # Manual names and an already-generated title always win.
            name = " ".join(name.split()).strip('\"')[:60].strip()
            if not name:
                raise ValueError("Conversation name cannot be empty")
        record["name"] = name
        record["name_source"] = "ai" if generated else "manual"
        self._publish(state)
        return True

    def branch_point(self, index):
        source = self.current
        messages = source["messages"]
        if type(index) is not int or not 0 <= index < len(messages):
            raise ValueError("Message not found")
        message = messages[index]
        if message["role"] not in ("You", "Loom"):
            raise ValueError("Choose a user or assistant message")
        position = "before" if message["role"] == "You" else "after"
        turn_id = message.get("turn_id")
        cut = index if position == "before" else index + 1
        indices = [i for i, m in enumerate(messages) if turn_id and m.get("turn_id") == turn_id]
        native = bool(source.get("thread_id") and indices and message.get("branch_point") == position
                      and cut == (min(indices) if position == "before" else max(indices) + 1)
                      and not (position == "before" and turn_id == source.get("replay_turn_id")))
        return {"conversation_id": source["id"], "message_index": index, "turn_id": turn_id,
                "position": position, "native": native, "messages": messages[:cut],
                "draft": message["text"] if position == "before" else ""}

    def branch(self, index, thread_id=None, *, replay=False):
        source = self.current
        point = self.branch_point(index)
        if replay:
            if thread_id is not None:
                raise ValueError("A replay branch must start a fresh native thread")
        elif (not isinstance(thread_id, str) or not thread_id.strip()
              or thread_id == source.get("thread_id") or not point["native"]):
            raise ValueError("A native branch requires an exact turn boundary and a separate provider thread")
        state = deepcopy(self.state)
        messages = point.pop("messages")
        draft = point.pop("draft")
        point.pop("native")
        point["method"] = "replay" if replay else "native"
        if replay:
            # These IDs belong to the original provider thread, not the fresh one.
            for message in messages:
                message.pop("turn_id", None)
                message.pop("branch_point", None)
        record = self._record(source["name"] + " — branch", {
            "messages": messages, "draft": draft,
            "thread_id": thread_id, "model": source.get("model"), "name_source": "manual",
            "replay_pending": bool(replay and messages), "replay_count": len(messages) if replay else 0,
            "replay_turn_id": None if replay else source.get("replay_turn_id"),
            "branched_from": point})
        state["records"].append(record)
        state["selected"] = record["id"]
        self._publish(state)
        return record["id"]

    def trim(self, index, *, edit=False, carried_usage=None):
        source = self.current
        messages = source["messages"]
        if type(index) is not int or not 0 <= index < len(messages):
            raise ValueError("Message not found")
        if edit and messages[index]["role"] != "You":
            raise ValueError("Choose a user prompt to edit")
        state = deepcopy(self.state)
        record = self._find(state, source["id"])
        prefix = messages[:index]
        for message in prefix:
            message.pop("turn_id", None)
            message.pop("branch_point", None)
        record.update(messages=prefix, thread_id=None, replay_count=len(prefix),
                      replay_pending=bool(prefix), replay_turn_id=None, history_trimmed=True,
                      context_epoch=uuid.uuid4().hex, legacy_proposals=False,
                      draft=messages[index]["text"] if edit else source.get("draft", ""))
        if carried_usage is not None:
            record["carried_usage"] = deepcopy(carried_usage)
        self._publish(state)

    @staticmethod
    def _ensure_selected(state):
        if not any(r["id"] == state["selected"] for r in state["records"]):
            active = next(iter(state["records"]), None)
            if active is None:
                active = Conversations._record("Conversation", {})
                state["records"].append(active)
            state["selected"] = active["id"]

    def delete(self, key):
        state = deepcopy(self.state)
        self._find(state, key)  # Reject unknown IDs without publishing.
        state["records"] = [r for r in state["records"] if r["id"] != key]
        self._ensure_selected(state)
        self._publish(state)


"""Offline API-equivalent estimates from Codex's cumulative usage notifications.

Standard USD/1M rates checked against https://developers.openai.com/api/docs/pricing
and /api/docs/models/<model> for Codex variants on 2026-11-14. Not subscription billing.
Each tuple is input, cached input, output, optional 30m cache write.
No network requests or provider calls are made here. Unknown models stay unknown.
"""
from __future__ import annotations

import hashlib
import math
import re

from .checkpoints.storage import atomic_json, read_json

RATES = {
    "gpt-6-astra": (10, 1, 50, 12.5),
    "gpt-6.1-sol": (2, .1, 10, 2.5),
    "gpt-6.1-terra": (1, .05, 5, 1.25),
    "gpt-6-luna": (.1, .01, .5, .125),
    "gpt-6-sol": (2, .2, 10, 2.5),
    "gpt-5.6-sol": (4, .4, 20, 5),
    "gpt-5.6-terra": (2, .2, 12, 2.5),
    "gpt-5.6-luna": (.2, .02, 1.2, .25),
    "gpt-5.5": (5, .5, 30),
    "gpt-5.5-pro": (30, None, 180),
    "gpt-5.4": (2.5, .25, 15),
    "gpt-5.4-pro": (30, None, 180),
    "gpt-5.4-mini": (.75, .075, 4.5),
    "gpt-5.4-nano": (.2, .02, 1.25),
    "gpt-5.3-codex": (1.75, .175, 14),
    "gpt-5.2-codex": (1.75, .175, 14),
    "gpt-5.1-codex": (1.25, .125, 10),
    "gpt-5.1-codex-max": (1.25, .125, 10),
    "gpt-5.1-codex-mini": (.25, .025, 2),
    "gpt-5.2": (1.75, .175, 14),
    "gpt-5.1": (1.25, .125, 10),
    "gpt-5": (1.25, .125, 10),
    "gpt-5-mini": (.25, .025, 2),
    "gpt-5-nano": (.05, .005, .4),
    "gpt-4.1": (2, .5, 8),
    "gpt-4.1-mini": (.4, .1, 1.6),
    "gpt-4o": (2.5, 1.25, 10),
    "gpt-4o-mini": (.15, .075, .6),
    "o3": (2, .5, 8),
}
LONG_CONTEXT = {name for name, rate in RATES.items() if len(rate) == 4} | {
    "gpt-5.5", "gpt-5.5-pro", "gpt-5.4", "gpt-5.4-pro"}
COUNTERS = ("inputTokens", "cachedInputTokens", "cacheWriteInputTokens", "outputTokens")


def estimate(model, counts, input_length):
    model = re.sub(r"-\d{4}-\d{2}-\d{2}$", "", model or "")
    rate = RATES.get(model)
    if rate is None:
        return None
    ordinary, cached_rate, output_rate = rate[:3]
    write_rate = rate[3] if len(rate) == 4 else None
    if model in LONG_CONTEXT and input_length > 272_000:
        ordinary *= 2
        cached_rate = cached_rate * 2 if cached_rate is not None else None
        output_rate *= 1.5
        write_rate = write_rate * 2 if write_rate is not None else None
    inputs, cached, writes, outputs = (counts.get(key, 0) for key in COUNTERS)
    if min(inputs, cached, writes, outputs) < 0 or cached + writes > inputs:
        return None
    if (cached and cached_rate is None) or (writes and write_rate is None):
        return None
    # Reasoning output is already included in outputTokens.
    return ((inputs - cached - writes) * ordinary + cached * (cached_rate or 0)
            + writes * (write_rate or 0) + outputs * output_rate) / 1_000_000


class ThreadUsage:
    def __init__(self, state=None, fresh=True):
        self.state = state or {"total": {}, "cost": 0.0, "complete": fresh,
                               "context_tokens": None, "context_window": None}

    @classmethod
    def load(cls, root, thread_id):
        if not thread_id:
            return cls()
        path = cls.path(root, thread_id)
        return cls(read_json(path) if path.exists() else None, fresh=False)

    @staticmethod
    def path(root, thread_id):
        key = hashlib.sha256(thread_id.encode("utf-8")).hexdigest()
        return root / "usage" / (key + ".json")

    def save(self, root, thread_id):
        if thread_id:
            atomic_json(self.path(root, thread_id), self.state)

    def update(self, usage, model):
        total = usage.get("total") or {}
        last = usage.get("last") or {}
        if not all(type(total.get(key)) is int and total[key] >= 0
                   for key in ("inputTokens", "cachedInputTokens", "outputTokens", "totalTokens")):
            return
        current = {key: total.get(key, 0) for key in COUNTERS}
        if any(type(value) is not int or value < 0 for value in current.values()):
            return
        previous = self.state["total"]
        delta = {key: current[key] - previous.get(key, 0) for key in COUNTERS}
        if any(value < 0 for value in delta.values()):
            # Counter resets/corrections cannot reconstruct historical model costs.
            self.state["complete"] = False
        elif any(delta.values()):
            cost = estimate(model, delta, last.get("inputTokens", 0))
            if cost is None:
                self.state["complete"] = False
            else:
                self.state["cost"] += cost
        self.state["total"] = current
        tokens, window = last.get("totalTokens"), usage.get("modelContextWindow")
        self.state["context_tokens"] = tokens if type(tokens) is int and tokens >= 0 else None
        self.state["context_window"] = window if type(window) is int and window > 0 else None

    def label(self):
        cost = "≈$%.2f" % self.state["cost"] if self.state["complete"] else "≈$—"
        tokens, window = self.state["context_tokens"], self.state["context_window"]
        context = ("C%d%%" % min(100, math.floor(tokens * 100 / window + .5))
                   if tokens is not None and window else "C—%")
        return cost + " " + context


TOOLTIP = ("Estimated standard API-equivalent thread cost, not subscription billing. "
           "C = latest reported context tokens / model context window (after compaction). "
           "Includes cached input; cache writes use the 30m rate when reported. "
           "Pricing snapshot: 2026-11-14; unknown pricing or untracked history shows —.")

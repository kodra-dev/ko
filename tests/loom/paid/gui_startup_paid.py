"""PAID: two real-model native GUI scenarios; never part of offline rendering tests."""
import json
import os
from pathlib import Path
import sys
import time

if os.environ.get("LOOM_ALLOW_PAID_TESTS") != "1":
    raise RuntimeError("PAID provider tests require explicit approval and -ConfirmPaidUsage.")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import hou
from gui_startup import GuiSmokeTest, log, DIRECTORY


class PaidGuiSmokeTest(GuiSmokeTest):
    def after_ui_checks(self):
        import loom
        self.pane = loom.open_pane()
        agent = self.pane.activeInterfaceRootWidget()._content
        from loom import provider
        from unittest.mock import patch
        with patch.object(provider, "data_root", return_value=Path(os.environ["LOOM_GUI_TEST_AUTH_ROOT"])):
            agent.connect_provider()
        self.live_started = time.monotonic()
        return False

    def extra_tick(self):
        if self.stage == 13:
            agent = self.pane.activeInterfaceRootWidget()._content
            assert time.monotonic() - self.live_started < 150, agent.status.text()
            if not agent.ready or not agent.model.count():
                assert agent.callbacks or agent.ready, agent.status.text()
                return
            self.live_start = len(agent.messages)
            agent.prompt.setPlainText("Do I have a shortcut to PolyExtrude now? Check my actual local setup.")
            agent.send_prompt()
            assert agent.busy
        elif self.stage == 14:
            agent = self.pane.activeInterfaceRootWidget()._content
            assert time.monotonic() - self.live_started < 150, agent.status.text()
            if agent.busy:
                return
            assert agent.status.text().startswith("Turn completed"), agent.status.text()
            messages = agent.messages[self.live_start:]
            assert any(m["role"] == "Tool" and "Alt+E" in m["text"] and "hotkeys" in m["text"]
                       for m in messages), repr(messages)
            answer = "\n".join(m["text"] for m in messages if m["role"] == "Loom")
            assert "alte" in "".join(c for c in answer.lower() if c.isalnum()), answer
            log("PASS real model + live local keymap: " + answer)
            self.pane.activeInterfaceRootWidget().grab().save(str(DIRECTORY / "live-hotkeys-chat.png"))
            hou.node("/obj/loom_gui_test/box1").setSelected(True, clear_all_selected=True)
            agent.allow_edits.setChecked(True)
            agent.prompt.setPlainText("Add a PolyExtrude after /obj/loom_gui_test/box1, distance 0.25, "
                                     "and display it. Do it now; metadata inspection is enough, no geometry cook request.")
            agent.send_prompt()
            assert agent.busy
        elif self.stage == 15:
            agent = self.pane.activeInterfaceRootWidget()._content
            assert time.monotonic() - self.live_started < 150, agent.status.text()
            if agent.busy:
                return
            assert agent.status.text().startswith("Turn completed"), agent.status.text()
            box = hou.node("/obj/loom_gui_test/box1")
            extrudes = [n for n in box.parent().children() if "polyextrude" in n.type().name()]
            assert any(box in n.inputs() and abs(n.parm("dist").eval() - 0.25) < 1e-6
                       and n.isDisplayFlagSet() for n in extrudes), repr(extrudes)
            assert any(p["status"] == "applied" for p in agent.tools.proposals.values())
            (DIRECTORY / "live-messages.json").write_text(json.dumps(agent.messages[self.live_start:], indent=2), encoding="utf-8")
            self.pane.activeInterfaceRootWidget().grab().save(str(DIRECTORY / "live-edits-chat.png"))
            log("PASS real model + automatic scene edits in native GUI")
            self.finish()
            return
        self.stage += 1


log("PAID: real provider requests; consumes ChatGPT subscription usage.")
test = PaidGuiSmokeTest()

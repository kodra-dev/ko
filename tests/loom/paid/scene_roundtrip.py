"""PAID: three real user turns through Panel, SceneTools and HOM. Explicit opt-in.

Creates its own headless Houdini process/scene; never touches an artist's open HIP.
Uses official existing login in place (no token access/copying). Keeps test artifacts.
"""
import json
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "python3.13libs"))
import hou
from PySide6 import QtTest, QtWidgets
from loom import provider
from loom.panel import Panel


def wait_until(predicate, panel, timeout=150):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        QtWidgets.QApplication.processEvents()
        if predicate():
            return
        if not panel.transport:
            raise RuntimeError(panel.status.text())
        QtTest.QTest.qWait(30)
    raise TimeoutError(panel.status.text() + "\n" + json.dumps(panel.messages[-4:]))


def turn(panel, prompt, edits=False):
    panel.allow_edits.setChecked(edits)
    panel.prompt.setPlainText(prompt)
    panel.send_prompt()
    assert panel.busy
    wait_until(lambda: not panel.busy, panel)
    assert not panel.error_details.toPlainText(), panel.error_details.toPlainText()
    assert panel.status.text().startswith("Turn completed"), panel.status.text()
    print("PASS:", prompt, flush=True)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="PAID: three real user turns, multiple scene tools. Consumes subscription usage.")
    parser.add_argument("--allow-paid-provider", action="store_true")
    if not parser.parse_args().allow_paid_provider:
        parser.error("Obtain user approval and pass --allow-paid-provider; this test costs usage.")
    # Capture the provider's normal private CODEX_HOME before isolating HIP storage.
    # Thread cwd/instructions/scene/history use only the disposable test directory.
    launch = provider.launch_config()
    directory = Path(tempfile.mkdtemp(prefix="loom-live-agent-"))
    print("Disposable HIP and conversation artifacts:", directory, flush=True)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = None
    with patch.dict(os.environ, {"LOOM_HOME": str(directory)}), \
         patch.object(provider, "launch_config", return_value=launch):
        try:
            hou.hipFile.clear(suppress_save_prompt=True)
            hou.hipFile.setName(str(directory / "working.hip"))
            geo = hou.node("/obj").createNode("geo", "cube", run_init_scripts=False)
            box = geo.createNode("box", "box1")
            box.setDisplayFlag(True)
            box.setRenderFlag(True)
            box.setSelected(True, clear_all_selected=True)
            panel = Panel()
            panel.connect_provider()
            wait_until(lambda: panel.ready and panel.model.count() > 0, panel, timeout=45)
            turn(panel, "Make /obj/cube/box1 wider: set its Size X to 2.5. Prepare the actual proposal, not just instructions.")
            assert box.parm("sizex").eval() == 1, "Edits-off turn modified the scene"
            assert any(p["status"] == "pending" for p in panel.tools.proposals.values())
            assert not panel.workspace.history.entries, "Proposal-only turn created checkpoints"
            turn(panel, "Add a PolyExtrude after /obj/cube/box1 with distance 0.25 and display its result. "
                 "Actually do it now. Do not apply the previous width proposal. Metadata inspection is enough; do not request a geometry cook.", edits=True)
            extrudes = [node for node in geo.children() if "polyextrude" in node.type().name()]
            assert extrudes, "No PolyExtrude was created"
            extrude = extrudes[-1]
            assert box in extrude.inputs(), "PolyExtrude is not connected to the cube"
            assert abs(extrude.parm("dist").eval() - 0.25) < 1e-6
            assert extrude.isDisplayFlagSet()
            assert box.parm("sizex").eval() == 1, "Old unapproved width proposal was applied"
            assert len(panel.workspace.history.entries) >= 2
            assert all(panel.workspace.history.verify(record["id"]).is_file()
                       for record in panel.workspace.history.entries)
            print("PASS: actual scene edits, wiring, parameters, display flag, and checkpoint files", flush=True)
            thread_id = panel.thread_id
            panel.shutdown()
            panel.deleteLater()
            app.processEvents()
            panel = Panel()
            assert panel.allow_edits.isChecked(), "Workspace edit preference was not restored"
            panel.connect_provider()
            wait_until(lambda: panel.ready and panel.model.count() > 0, panel, timeout=45)
            start = len(panel.messages)
            turn(panel, "Inspect the live PolyExtrude node now and tell me its current extrusion distance. Use loom_scene, not memory.")
            assert panel.thread_id == thread_id, "Conversation was replaced instead of resumed"
            latest = panel.messages[start:]
            assert "0.25" in "\n".join(m["text"] for m in latest if m["role"] == "Loom")
            assert any(m["role"] == "Tool" and m.get("success") for m in latest), latest
            print("PASS: durable conversation resumed with working tools and workspace edit preference", flush=True)
            path = extrude.path()
            applied = max((p for p in panel.tools.proposals.values() if p["status"] == "applied"
                           and any(s["op"] == "set" and s["path"] == path and s["parm"] == "dist"
                                   for s in p["steps"])), key=lambda p: p["created"])
            before = panel.workspace.history.verify(applied["baseline"])
            after = panel.workspace.history.verify(applied["result"])
            panel.shutdown()
            hou.hipFile.load(str(before), suppress_save_prompt=True, ignore_load_warnings=True)
            baseline_node = hou.node(path)
            assert baseline_node is None or abs(baseline_node.parm("dist").eval() - 0.25) > 1e-6
            hou.hipFile.load(str(after), suppress_save_prompt=True, ignore_load_warnings=True)
            assert abs(hou.node(path).parm("dist").eval() - 0.25) < 1e-6
            print("PASS: checkpoint HIP contents are genuinely before and after the edit", flush=True)
        finally:
            if panel:
                (directory / "messages.json").write_text(json.dumps(panel.messages, indent=2), encoding="utf-8")
                panel.shutdown()
                panel.deleteLater()
                app.processEvents()
            hou.hipFile.clear(suppress_save_prompt=True)


if __name__ == "__main__":
    main()

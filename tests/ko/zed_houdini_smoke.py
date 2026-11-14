"""Run explicitly with hython. No GUI, Zed launch, provider, or saved scene changes."""
from pathlib import Path
import sys
import tempfile
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
import hou
import ko_zed as zed


def main():
    hou.setUpdateMode(hou.updateMode.Manual)
    geo = hou.node("/obj").createNode("geo", "ko_zed_smoke")
    try:
        cases = [("attribwrangle", "snippet", ".vfl"), ("python", "python", ".py"),
                 ("opencl", "kernelcode", ".cl"), ("apex::script", "snippet", ".apex")]
        with tempfile.TemporaryDirectory() as directory:
            bridge = zed.Bridge.__new__(zed.Bridge)
            bridge.root = Path(directory)
            bridge.bindings = {}
            bridge.launcher = Mock()
            # Headless HOM lacks hou.ui; only replace the nonessential status notification.
            with patch.object(hou, "ui", Mock(), create=True):
                for node_type, name, extension in cases:
                    node = geo.createNode(node_type)
                    parm = node.parm(name)
                    assert parm in zed.code_parms(node)
                    parm.set("initial")
                    path = bridge.attach(parm)
                    assert path.suffix == extension, (node_type, path)
                    text = '// $HIP and `literal` and Unicode é\n'
                    path.write_text(text, encoding="utf-8")
                    binding = bridge.bindings[(node.sessionId(), name)]
                    binding.poll(zed.Bridge.apply)
                    binding.poll(zed.Bridge.apply)
                    assert not binding.paused, binding.paused
                    assert parm.unexpandedString() == text
                    # A node rename preserves the binding, independent of current selection/path.
                    node.setName(node.name() + "_renamed")
                    assert bridge.attach(parm) == path
                    parm.set("Houdini revision")
                    path.write_text("Zed revision", encoding="utf-8")
                    binding.poll(zed.Bridge.apply)
                    binding.poll(zed.Bridge.apply)
                    assert binding.paused.startswith("Conflict")
                    assert parm.unexpandedString() == "Houdini revision"
                    assert path.read_text(encoding="utf-8") == "Zed revision"
                    print("PASS", node_type, name, extension)
                bridge.scene_event(hou.hipFileEventType.BeforeClear)
                assert not bridge.bindings
        print("PASS real HOM synchronization, raw strings, rename, conflicts, scene detach")
    finally:
        geo.destroy()


if __name__ == "__main__":
    main()

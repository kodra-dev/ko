"""UI-thread inspection helpers. No modal approvals and no unsafe HOM worker threads."""
from contextlib import contextmanager


@contextmanager
def native_operation(label, progress):
    import hou
    progress()
    if not hou.isUIAvailable():
        yield lambda: progress()
        return
    # Native cooks can check Houdini's Esc interrupt flag internally. Python Stop
    # requests are checked between calls; uncooperative plugin cooks cannot be
    # forcibly killed safely in-process.
    with hou.InterruptableOperation(label, open_interrupt_dialog=False) as operation:
        def tick():
            operation.updateProgress()
            progress()
        tick()
        yield tick
        tick()


def viewport_image(progress=lambda: None):
    """Grab the already displayed Scene Viewer, not a flipbook/render/scene save."""
    import hou
    from PySide6 import QtCore
    if not hou.isUIAvailable():
        raise RuntimeError("Viewport capture requires graphical Houdini and a visible Scene Viewer.")
    progress()
    viewer = next((pane for pane in hou.ui.paneTabs()
                   if pane.type() == hou.paneTabType.SceneViewer and pane.isCurrentTab()), None)
    if viewer is None:
        raise RuntimeError("Open a Scene Viewer before requesting a viewport image.")
    widget = viewer.qtWindow()
    if not widget.isVisible():
        raise RuntimeError("The Scene Viewer is not visible.")
    pixmap = widget.grab()
    if pixmap.isNull():
        raise RuntimeError("Houdini returned an empty viewport capture.")
    pixmap = pixmap.scaled(960, 960, QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                           QtCore.Qt.TransformationMode.SmoothTransformation)
    buffer = QtCore.QBuffer()
    buffer.open(QtCore.QIODevice.OpenModeFlag.WriteOnly)
    if not pixmap.save(buffer, "PNG"):
        raise RuntimeError("Could not encode the viewport image.")
    progress()
    return {"source": "Currently displayed Scene Viewer (including its HUD/toolbars)",
            "frame": hou.frame(), "width": pixmap.width(), "height": pixmap.height(),
            "update_mode": str(hou.updateModeSetting()),
            "note": "No render/flipbook or forced cook. In Manual update mode the displayed image may be stale.",
            "_image_url": "data:image/png;base64," + bytes(buffer.data().toBase64()).decode("ascii")}

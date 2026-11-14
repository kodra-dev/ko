"""Shared checkpoint dialogs; importing these never starts the agent."""
import difflib
import json
from PySide6 import QtCore, QtGui, QtWidgets
from .scene import graph_snapshot
from .storage import graph_diff

def button(label, callback, layout):
    widget = QtWidgets.QPushButton(label)
    widget.clicked.connect(callback)
    layout.addWidget(widget)
    return widget


def pretty(value):
    return json.dumps(value, indent=2, ensure_ascii=False, default=str)


class TextInputDialog(QtWidgets.QDialog):
    """Explicit keyboard handling so Houdini cannot claim the name-field Enter."""
    def __init__(self, parent, title, label, text=""):
        super().__init__(parent)
        self.setWindowTitle(title)
        layout = QtWidgets.QVBoxLayout(self)
        caption = QtWidgets.QLabel(label)
        caption.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        layout.addWidget(caption)
        self.input = QtWidgets.QLineEdit(text)
        layout.addWidget(self.input)
        self.buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        self.buttons.button(QtWidgets.QDialogButtonBox.StandardButton.Ok).setDefault(True)
        self.input.returnPressed.connect(self.accept)
        layout.addWidget(self.buttons)
        self.input.installEventFilter(self)
        self.input.selectAll()
        self.input.setFocus()

    def eventFilter(self, watched, event):
        if watched is self.input and event.type() in (QtCore.QEvent.Type.ShortcutOverride,
                                                      QtCore.QEvent.Type.KeyPress):
            key = event.key()
            if key in (QtCore.Qt.Key.Key_Return, QtCore.Qt.Key.Key_Enter, QtCore.Qt.Key.Key_Escape):
                event.accept()
                if event.type() == QtCore.QEvent.Type.KeyPress:
                    self.reject() if key == QtCore.Qt.Key.Key_Escape else self.accept()
                return True
        return super().eventFilter(watched, event)


def prompt_text(parent, title, label, text=""):
    dialog = TextInputDialog(parent, title, label, text)
    try:
        accepted = dialog.exec() == QtWidgets.QDialog.DialogCode.Accepted
        return dialog.input.text(), accepted
    finally:
        dialog.deleteLater()


class CompareDialog(QtWidgets.QDialog):
    def __init__(self, workspace, before_id, after_id=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Checkpoints — Compare (saved captures are not live views)")
        self.resize(1100, 760)
        layout = QtWidgets.QVBoxLayout(self)
        before = workspace.history.get(before_id)
        after = workspace.history.get(after_id) if after_id else None
        layout.addWidget(QtWidgets.QLabel(before["label"] + "  →  " + (after["label"] if after else "Current live graph")))
        layout.addWidget(QtWidgets.QLabel("Graph diffs exclude geometry payloads, external assets, and untracked node state."))
        tabs = QtWidgets.QTabWidget()
        layout.addWidget(tabs)
        a = workspace.graph(before_id)
        b = workspace.graph(after_id) if after_id else graph_snapshot()
        diff = graph_diff(a, b)
        summary = QtWidgets.QPlainTextEdit()
        summary.setReadOnly(True)
        summary.setPlainText(pretty(diff))
        tabs.addTab(summary, "Scene changes")
        text = QtWidgets.QPlainTextEdit()
        text.setReadOnly(True)
        lines = []
        for path in sorted(a["nodes"].keys() | b["nodes"].keys()):
            old, new = a["nodes"].get(path), b["nodes"].get(path)
            if old != new:
                lines.extend(difflib.unified_diff(pretty(old).splitlines(), pretty(new).splitlines(),
                             fromfile=before["label"] + ":" + path,
                             tofile=(after["label"] if after else "Live") + ":" + path, lineterm=""))
        text.setPlainText("\n".join(lines) or "No tracked node changes.")
        tabs.addTab(text, "Text / VEX / parameters")
        previews = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(previews)
        for record in (before, after):
            column = QtWidgets.QVBoxLayout()
            row.addLayout(column)
            label = QtWidgets.QLabel(record["label"] if record else "Live state — checkpoint it to capture a preview")
            label.setWordWrap(True)
            column.addWidget(label)
            image = QtWidgets.QLabel("No captured preview")
            image.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            if record and record.get("preview"):
                path = workspace.history.directory(record["id"]) / record["preview"]
                pixmap = QtGui.QPixmap(str(path))
                image.setPixmap(pixmap.scaled(500, 400, QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                                              QtCore.Qt.TransformationMode.SmoothTransformation))
                label.setText(label.text() + "\nFrame: " + str(record["context"]["frame"]))
            column.addWidget(image, 1)
        tabs.addTab(previews, "Captured previews")
        layout.addWidget(QtWidgets.QLabel("Compare matching camera/frame/display settings. Double-click a checkpoint to switch. Changes save into the checkpoint you leave."))
        button("Close", self.accept, layout)



"""Compact permission selector: three stops and only the current mode's label."""
from PySide6 import QtCore, QtGui, QtWidgets

MODES = ("read_only", "scene_edit", "tool_dev")
LABELS = ("Read Only", "Scene Edit", "Tool Dev")
DESCRIPTIONS = (
    "Inspect and explain; do not change the scene or source files.",
    "Change the scene and control Houdini's UI, including Python. Scene changes use checkpoints; UI changes do not.",
    "Scene/UI editing plus persistent scripts, panels and ko source edits. Files are not restored by HIP checkpoints.",
)


class ModeSlider(QtWidgets.QSlider):
    def __init__(self, parent=None):
        super().__init__(QtCore.Qt.Orientation.Horizontal, parent)
        self.setRange(0, 2)
        self.setPageStep(1)
        self.setFixedSize(66, 24)
        self.setAccessibleName("Loom permission mode")

    def _position(self, index):
        if self.layoutDirection() == QtCore.Qt.LayoutDirection.RightToLeft:
            index = 2 - index
        return 10 + index * (self.width() - 20) / 2

    def _index(self, x):
        return min(range(3), key=lambda i: abs(x - self._position(i)))

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        palette = self.palette()
        y = self.height() / 2
        # Mid can nearly disappear against Houdini's dark pane background. Use
        # foreground-derived contrast for the whole control, not just its selection.
        track = palette.color(QtGui.QPalette.ColorRole.WindowText)
        track.setAlpha(140 if self.isEnabled() else 65)
        outline = palette.color(QtGui.QPalette.ColorRole.WindowText)
        outline.setAlpha(200 if self.isEnabled() else 90)
        painter.setPen(QtGui.QPen(track, 2, QtCore.Qt.PenStyle.SolidLine,
                                 QtCore.Qt.PenCapStyle.RoundCap))
        painter.drawLine(QtCore.QPointF(10, y), QtCore.QPointF(self.width() - 10, y))
        for index in range(3):
            selected = index == self.value()
            painter.setPen(QtGui.QPen(palette.color(QtGui.QPalette.ColorRole.Mid), 1)
                           if selected else QtGui.QPen(outline, 1.8))
            painter.setBrush(palette.brush(QtGui.QPalette.ColorRole.Highlight if selected
                                           else QtGui.QPalette.ColorRole.Window))
            radius = 4.5 if selected else 5
            painter.drawEllipse(QtCore.QPointF(self._position(index), y), radius, radius)
        if self.hasFocus():
            option = QtWidgets.QStyleOptionFocusRect()
            option.initFrom(self)
            option.rect = self.rect().adjusted(1, 1, -1, -1)
            self.style().drawPrimitive(QtWidgets.QStyle.PrimitiveElement.PE_FrameFocusRect, option, painter, self)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.setFocus(QtCore.Qt.FocusReason.MouseFocusReason)
            self.setValue(self._index(event.position().x()))
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        # A deliberate click selects a mode; dragging cannot accidentally escalate it.
        event.accept()

    def wheelEvent(self, event):
        event.ignore()

    def event(self, event):
        if event.type() == QtCore.QEvent.Type.ToolTip:
            index = self._index(event.pos().x())
            QtWidgets.QToolTip.showText(event.globalPos(), LABELS[index] + "\n" + DESCRIPTIONS[index], self)
            return True
        return super().event(event)


class ModeControl(QtWidgets.QWidget):
    modeChanged = QtCore.Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.slider = ModeSlider(self)
        self.label = QtWidgets.QLabel(self)
        # Reserve only the longest current-state label, avoiding row movement.
        self.label.setMinimumWidth(max(self.label.fontMetrics().horizontalAdvance(s) for s in LABELS))
        layout.addWidget(self.slider)
        layout.addWidget(self.label)
        self.setFocusProxy(self.slider)
        self.slider.valueChanged.connect(self._changed)
        self._changed(0)

    def mode(self):
        return MODES[self.slider.value()]

    def setMode(self, mode):
        self.slider.setValue(MODES.index(mode))

    def _changed(self, index):
        self.label.setText(LABELS[index])
        self.label.setToolTip(DESCRIPTIONS[index])
        self.slider.setAccessibleDescription(LABELS[index] + ". " + DESCRIPTIONS[index])
        self.modeChanged.emit(MODES[index])

    # Compatibility for existing scripted panel clients using the former checkbox.
    def isChecked(self):
        return self.mode() != "read_only"

    def setChecked(self, checked):
        self.setMode("scene_edit" if checked else "read_only")

    def text(self):
        return self.label.text()

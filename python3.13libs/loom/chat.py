"""Native Qt chat cards and composer; no webview or executable message markup."""
import math
from PySide6 import QtCore, QtGui, QtWidgets


def larger_font(widget):
    font = widget.font()
    font.setPointSizeF(max(11.0, font.pointSizeF() + 1.5))
    widget.setFont(font)
    return font


class BusySpinner(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(20, 20)
        self.angle = 0
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(70)
        self.timer.timeout.connect(self._advance)
        self.hide()

    def _advance(self):
        self.angle = (self.angle + 30) % 360
        self.update()

    def set_running(self, running):
        self.setVisible(running)
        self.timer.start() if running else self.timer.stop()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.setPen(QtGui.QPen(QtGui.QColor("#b4c6e4"), 2.5))
        painter.drawArc(QtCore.QRectF(3, 3, 14, 14), -self.angle * 16, 250 * 16)


def action_icon(copy=False):
    pixmap = QtGui.QPixmap(18, 18)
    pixmap.fill(QtCore.Qt.GlobalColor.transparent)
    painter = QtGui.QPainter(pixmap)
    painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
    painter.setPen(QtGui.QPen(QtGui.QColor("#dce3ef"), 1.5))
    if copy:
        painter.drawRoundedRect(QtCore.QRectF(6, 6, 9, 10), 1, 1)
        painter.drawLine(3, 12, 3, 3)
        painter.drawLine(3, 3, 11, 3)
    else:
        painter.drawLine(4, 5, 14, 5)
        painter.drawLine(4, 9, 14, 9)
        painter.drawLine(4, 13, 11, 13)
    painter.end()
    return QtGui.QIcon(pixmap)


class Composer(QtWidgets.QPlainTextEdit):
    submitted = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        larger_font(self)
        self._composing = False
        self.setObjectName("loomComposer")
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.setTabChangesFocus(True)
        self.setPlaceholderText("Ask about your scene or describe a change…")
        self.setMinimumHeight(76)
        self.setMaximumHeight(120)
        self.setStyleSheet("QPlainTextEdit#loomComposer {padding: 8px; border: 1px solid #606875; border-radius: 7px;} "
                           "QPlainTextEdit#loomComposer:focus {border: 2px solid #9baed4;}")

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            # Native floating Python Panels may activate their container first.
            # Make that first click both activate the window and place the caret.
            self.window().activateWindow()
            self.setFocus(QtCore.Qt.FocusReason.MouseFocusReason)
        super().mousePressEvent(event)
        event.accept()

    def event(self, event):
        if (event.type() == QtCore.QEvent.Type.ShortcutOverride
                and event.key() in (QtCore.Qt.Key.Key_Return, QtCore.Qt.Key.Key_Enter)):
            # Houdini intercepts Return unless we claim it before its hotkey layer.
            event.accept()
            return True
        return super().event(event)

    def inputMethodEvent(self, event):
        self._composing = bool(event.preeditString())
        super().inputMethodEvent(event)

    def keyPressEvent(self, event):
        if event.key() in (QtCore.Qt.Key.Key_Return, QtCore.Qt.Key.Key_Enter) and not self._composing:
            event.accept()
            if event.modifiers() & QtCore.Qt.KeyboardModifier.ShiftModifier:
                self.insertPlainText("\n")
            elif not event.isAutoRepeat():
                self.submitted.emit()
            return
        super().keyPressEvent(event)


class MessageBody(QtWidgets.QTextBrowser):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.document().setDefaultFont(larger_font(self))
        self.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.setOpenLinks(False)
        self.setOpenExternalLinks(False)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Fixed)
        self.setStyleSheet("QTextBrowser {background: transparent; color: #e5e7eb; border: none;}")
        self.limit_height = None
        self.document().setDocumentMargin(2)
        self.document().setDefaultStyleSheet("pre, code {font-family: Consolas, monospace;} p {margin: 6px 0;}")
        self.document().documentLayout().documentSizeChanged.connect(self._fit)

    def loadResource(self, resource_type, url):
        # Neither markdown images nor raw HTML may fetch local/network resources.
        return None

    def set_content(self, text, markdown=True):
        if markdown:
            features = QtGui.QTextDocument.MarkdownFeature
            self.document().setMarkdown(text, features.MarkdownDialectGitHub | features.MarkdownNoHTML)
        else:
            self.setPlainText(text)
        self._fit()

    def _fit(self, *_):
        height = max(28, math.ceil(self.document().size().height()) + 8)
        self.setFixedHeight(min(height, self.limit_height) if self.limit_height else height)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.document().setTextWidth(max(20, self.viewport().width()))
        self._fit()


class MessageCard(QtWidgets.QFrame):
    def __init__(self, message, parent=None):
        super().__init__(parent)
        self.setObjectName("loomMessageCard")
        self.setStyleSheet('''
QFrame#loomMessageCard {background: #292b30; border: 1px solid #454951; border-radius: 7px;}
QFrame#loomMessageCard[kind="user"] {background: #344052; border-color: #566780;}
QFrame#loomMessageCard[kind="activity"] {background: #26282b;}
QFrame#loomMessageCard[kind="error"] {border-color: #b76b67;}
QLabel {color: #b9c5db; border: none; background: transparent;}
QToolButton {border: none; background: transparent; color: #c7cbd3; padding: 2px;}
''')
        self._message = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 9, 12, 9)
        self.heading = QtWidgets.QWidget(self)
        heading = QtWidgets.QHBoxLayout(self.heading)
        heading.setContentsMargins(0, 0, 58, 0)
        layout.addWidget(self.heading)
        self.title = QtWidgets.QLabel()
        self.title.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.title.setWordWrap(True)
        font = self.title.font()
        font.setBold(True)
        self.title.setFont(font)
        heading.addWidget(self.title, 1)
        self.actions = QtWidgets.QWidget(self)
        self.actions.setObjectName("loomMessageActions")
        self.actions.setStyleSheet("QWidget#loomMessageActions {background: #383d47; border-radius: 4px;}")
        actions = QtWidgets.QHBoxLayout(self.actions)
        actions.setContentsMargins(3, 2, 3, 2)
        actions.setSpacing(2)
        self.toggle = QtWidgets.QToolButton(self.actions)
        self.toggle.setCheckable(True)
        self.toggle.setIcon(action_icon())
        self.toggle.setToolTip("Show / hide details")
        self.toggle.setAccessibleName("Details")
        actions.addWidget(self.toggle)
        self.copy_button = QtWidgets.QToolButton(self.actions)
        self.copy_button.setIcon(action_icon(copy=True))
        self.copy_button.setToolTip("Copy message")
        self.copy_button.setAccessibleName("Copy")
        self.copy_button.clicked.connect(lambda: QtWidgets.QApplication.clipboard().setText(self._message["text"]))
        actions.addWidget(self.copy_button)
        self.actions.hide()
        self.body = MessageBody(self)
        self.body.setViewportMargins(0, 0, 58, 0)
        layout.addWidget(self.body)
        self.toggle.toggled.connect(self.body.setVisible)
        self.update_message(message)

    def enterEvent(self, event):
        self.actions.adjustSize()
        self.actions.move(max(2, self.width() - self.actions.width() - 6), 5)
        self.actions.raise_()
        self.actions.show()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.actions.hide()
        super().leaveEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.actions.move(max(2, self.width() - self.actions.width() - 6), 5)

    def update_message(self, message):
        if message == self._message:
            return
        first = self._message is None
        was_failed = self._message is not None and self._message.get("success") is False
        self._message = dict(message)
        role = message["role"]
        activity = role in ("Tool", "Application result", "System")
        error = role == "Error" or message.get("success") is False
        self.setProperty("kind", "error" if error else "user" if role == "You" else "activity" if activity else "agent")
        self.style().unpolish(self)
        self.style().polish(self)
        self.title.setText(message.get("summary") or role)
        self.heading.setVisible(activity or error)
        self.toggle.setVisible(activity)
        if first or (error and not was_failed):
            self.toggle.setChecked(error)
        self.body.limit_height = 320 if activity else None
        self.body.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded if activity
                                             else QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.body.set_content(message["text"], markdown=not activity and role != "Error")
        self.body.setVisible(not activity or self.toggle.isChecked())


class Transcript(QtWidgets.QScrollArea):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.content = QtWidgets.QWidget()
        self.stack = QtWidgets.QVBoxLayout(self.content)
        self.stack.setContentsMargins(0, 6, 0, 6)
        self.stack.setSpacing(12)
        self.empty = QtWidgets.QLabel("Your Houdini workspace, in conversation.\n\nAsk about the scene or your local tools.\nChanges stay proposals unless you allow scene edits.")
        self.empty.setWordWrap(True)
        self.empty.setStyleSheet("padding: 24px; color: #b1b8c6;")
        self.stack.addWidget(self.empty)
        self.stack.addStretch()
        self.setWidget(self.content)
        self.cards = []
        self._follow = True
        self.verticalScrollBar().valueChanged.connect(
            lambda value: setattr(self, "_follow", value >= self.verticalScrollBar().maximum() - 30))
        self.verticalScrollBar().rangeChanged.connect(self._range_changed)

    def _range_changed(self, *_):
        if self._follow:
            self.verticalScrollBar().setValue(self.verticalScrollBar().maximum())

    def set_messages(self, messages):
        # Keep the durable log unchanged; collapse consecutive tool activity only
        # in the view, like other harnesses' expandable action summaries.
        grouped = []
        for message in messages:
            if message["role"] == "Tool" and grouped and isinstance(grouped[-1], list):
                grouped[-1].append(message)
            else:
                grouped.append([message] if message["role"] == "Tool" else message)
        view = []
        for group in grouped:
            if not isinstance(group, list):
                view.append(group)
            elif len(group) == 1:
                view.append(group[0])
            else:
                edited = sum(m.get("summary", "").startswith("Edited scene") for m in group)
                failed = sum(m.get("success") is False for m in group)
                summary = str(len(group)) + " actions"
                if edited:
                    summary += " · " + str(edited) + " scene edits"
                if failed:
                    summary += " · " + str(failed) + " failed"
                view.append({"role": "Tool", "summary": summary, "success": not failed,
                             "text": "\n\n".join(m.get("summary", "Tool") + "\n" + m["text"] for m in group)})
        messages = view
        bar = self.verticalScrollBar()
        self._follow = bar.value() >= bar.maximum() - 30
        while len(self.cards) > len(messages):
            self.cards.pop().deleteLater()
        self.empty.setVisible(not messages)
        for index, message in enumerate(messages):
            if index == len(self.cards):
                card = MessageCard(message)
                self.cards.append(card)
                self.stack.insertWidget(self.stack.count() - 1, card)
            else:
                self.cards[index].update_message(message)
        if self._follow:
            bar.setValue(bar.maximum())

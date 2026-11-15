"""Small modeless conversation manager, using Checkpoints' active/selection styling."""
from PySide6 import QtCore, QtGui, QtWidgets

from .checkpoints.panel import CheckpointDelegate, CURRENT_ROLE
from .checkpoints.ui import button, prompt_text

PREVIEW_ROLE = CURRENT_ROLE + 1


def new_conversation_button(callback, layout):
    control = QtWidgets.QToolButton()
    control.setToolTip("New conversation")
    control.setAccessibleName("New conversation")
    control.setAutoRaise(True)
    icon = QtGui.QIcon()
    for mode, group in ((QtGui.QIcon.Mode.Normal, QtGui.QPalette.ColorGroup.Active),
                        (QtGui.QIcon.Mode.Disabled, QtGui.QPalette.ColorGroup.Disabled)):
        pixmap = QtGui.QPixmap(32, 32)
        pixmap.setDevicePixelRatio(2)
        pixmap.fill(QtCore.Qt.GlobalColor.transparent)
        painter = QtGui.QPainter(pixmap)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.setPen(QtGui.QPen(control.palette().color(group, QtGui.QPalette.ColorRole.ButtonText),
                                 1.8, QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenCapStyle.RoundCap))
        painter.drawLine(QtCore.QPointF(3, 8), QtCore.QPointF(13, 8))
        painter.drawLine(QtCore.QPointF(8, 3), QtCore.QPointF(8, 13))
        painter.end()
        icon.addPixmap(pixmap, mode)
    control.setIcon(icon)
    control.setIconSize(QtCore.QSize(16, 16))
    control.setMinimumSize(24, 24)
    control.clicked.connect(callback)
    layout.addWidget(control)
    return control


def first_line(record):
    for message in record["messages"]:
        if message.get("role") == "You":
            return next((line.strip() for line in message.get("text", "").splitlines() if line.strip()), "")
    return ""


class ConversationDelegate(CheckpointDelegate):
    def initStyleOption(self, option, index):
        super().initStyleOption(option, index)
        option.text = ""  # The shared delegate draws just the background/selection.

    def sizeHint(self, option, index):
        return QtCore.QSize(180, QtGui.QFontMetrics(option.font).height() * 2 + 16)

    def paint(self, painter, option, index):
        super().paint(painter, option, index)
        painter.save()
        rect = option.rect.adjusted(10, 6, -10, -6)
        painter.setClipRect(rect)
        font = QtGui.QFont(option.font)
        font.setBold(bool(index.data(CURRENT_ROLE)))
        painter.setFont(font)
        color = option.palette.color(QtGui.QPalette.ColorRole.HighlightedText if index.data(CURRENT_ROLE)
                                     else QtGui.QPalette.ColorRole.Text)
        painter.setPen(color)
        metrics = QtGui.QFontMetrics(font)
        painter.drawText(rect.left(), rect.top() + metrics.ascent(),
                         metrics.elidedText(index.data() or "", QtCore.Qt.TextElideMode.ElideRight, rect.width()))
        font.setBold(False)
        painter.setFont(font)
        color.setAlpha(145)
        painter.setPen(color)
        metrics = QtGui.QFontMetrics(font)
        painter.drawText(rect.left(), rect.top() + metrics.height() + 4 + metrics.ascent(),
                         metrics.elidedText(index.data(PREVIEW_ROLE) or "", QtCore.Qt.TextElideMode.ElideRight, rect.width()))
        painter.restore()


class ConversationManager(QtWidgets.QDialog):
    def __init__(self, owner):
        super().__init__(owner)
        self.owner = owner
        self.setObjectName("loom_conversations_manager")
        self.setWindowTitle("Conversations")
        self.resize(480, 540)
        layout = QtWidgets.QVBoxLayout(self)
        toolbar = QtWidgets.QHBoxLayout()
        layout.addLayout(toolbar)
        self.new_button = new_conversation_button(lambda: owner._guard(owner.new_chat), toolbar)
        toolbar.addStretch()
        self.delete_button = button("Delete", lambda: owner._guard(self.delete_selected), toolbar)
        self.delete_button.setToolTip("Permanently remove the selected Loom conversation. Does not delete scene checkpoints or native provider files.")
        self.items = QtWidgets.QTreeWidget()
        self.items.setHeaderHidden(True)
        self.items.setRootIsDecorated(False)
        self.items.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
        self.items.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.items.setItemDelegate(ConversationDelegate(self.items))
        self.items.itemDoubleClicked.connect(lambda *_: owner._guard(self.activate_selected))
        self.items.itemSelectionChanged.connect(self.update_actions)
        self.items.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.CustomContextMenu)
        self.items.customContextMenuRequested.connect(self.context_menu)
        layout.addWidget(self.items, 1)
        for widget in (self, self.items, self.items.viewport()):
            widget.installEventFilter(self)
        self.refresh()

    def selected_id(self):
        selected = self.items.selectedItems()
        return selected[0].data(0, QtCore.Qt.ItemDataRole.UserRole) if selected else None

    def refresh(self):
        selected = self.selected_id()
        records = self.owner.conversations.records
        visible_ids = {r["id"] for r in records}
        if getattr(self, "_active", None) != self.owner.conversation_id or selected not in visible_ids:
            selected = self.owner.conversation_id
        self._active = self.owner.conversation_id
        self.items.clear()
        for record in records:
            item = QtWidgets.QTreeWidgetItem([record["name"]])
            item.setData(0, QtCore.Qt.ItemDataRole.UserRole, record["id"])
            item.setData(0, CURRENT_ROLE, record["id"] == self.owner.conversation_id)
            item.setData(0, PREVIEW_ROLE, first_line(record))
            item.setToolTip(0, record["name"] + "\n" + first_line(record))
            self.items.addTopLevelItem(item)
            if record["id"] == (selected or self.owner.conversation_id):
                self.items.setCurrentItem(item)
        self.update_actions()

    def update_actions(self):
        idle = not self.owner.busy and self.owner.session.turn_owner is None
        selected = self.items.selectedItems()
        self.new_button.setEnabled(idle)
        self.delete_button.setEnabled(idle and bool(selected))

    def _valid_selection(self):
        workspace = self.owner._bound_workspace
        self.owner._ensure_binding()
        if workspace != self.owner._bound_workspace:
            raise RuntimeError("Scene changed; select a conversation in the current workspace.")
        if self.owner.busy or self.owner.session.turn_owner is not None:
            return None
        return self.selected_id()

    def activate_selected(self):
        key = self._valid_selection()
        if not key:
            return
        self.owner.switch_conversation(key)

    def rename_selected(self):
        key = self._valid_selection()
        if not key:
            return
        store = self.owner.conversations
        record = next(r for r in store.records if r["id"] == key)
        name, accepted = prompt_text(self, "Rename conversation", "Name", record["name"])
        if accepted and not self.owner.busy:
            self.owner._ensure_binding()
            if self.owner.conversations.path != store.path:
                raise RuntimeError("Scene changed; rename cancelled.")
            self.owner.conversations.rename(key, name)
            self.owner._refresh_conversations()

    def copy_selected(self):
        key = self._valid_selection()
        if not key:
            return
        record = next(r for r in self.owner.conversations.records if r["id"] == key)
        text = "\n\n".join([record["name"]] + [
            f"{message['role']}:\n{message.get('text', '')}" for message in record["messages"]])
        QtWidgets.QApplication.clipboard().setText(text)

    def delete_selected(self):
        key = self._valid_selection()
        if key:
            self.owner.delete_conversation(key)

    def context_menu(self, position):
        item = self.items.itemAt(position)
        if item:
            self.items.setCurrentItem(item)
        if not self.selected_id():
            return
        menu = QtWidgets.QMenu(self)
        for label, callback in (("Activate", self.activate_selected), ("Rename…  F2", self.rename_selected),
                                ("Copy as text", self.copy_selected),
                                ("Delete", self.delete_selected)):
            menu.addAction(label, lambda checked=False, callback=callback: self.owner._guard(callback)).setEnabled(
                not self.owner.busy and self.owner.session.turn_owner is None)
        menu.exec(self.items.viewport().mapToGlobal(position))
        menu.deleteLater()

    def eventFilter(self, watched, event):
        if (event.type() in (QtCore.QEvent.Type.ShortcutOverride, QtCore.QEvent.Type.KeyPress)
                and QtWidgets.QApplication.activeModalWidget() is None
                and event.modifiers() == QtCore.Qt.KeyboardModifier.NoModifier):
            callback = {QtCore.Qt.Key.Key_F2: self.rename_selected,
                        QtCore.Qt.Key.Key_Delete: self.delete_selected,
                        QtCore.Qt.Key.Key_Return: self.activate_selected,
                        QtCore.Qt.Key.Key_Enter: self.activate_selected}.get(event.key())
            if callback:
                event.accept()
                if event.type() == QtCore.QEvent.Type.KeyPress and not event.isAutoRepeat():
                    self.owner._guard(callback)
                return True
        return super().eventFilter(watched, event)

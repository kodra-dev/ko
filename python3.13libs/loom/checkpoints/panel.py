"""Independent editable Checkpoints pane. No agent/provider dependency."""
from datetime import datetime
import traceback
from PySide6 import QtCore, QtGui, QtWidgets
from .service import acquire
from .actions import registered_actions
from .ui import CompareDialog, button, pretty, prompt_text

CURRENT_ROLE = int(QtCore.Qt.ItemDataRole.UserRole) + 1


def ai_icon():
    """Small vector sparkle, independent of agent imports and font glyph support."""
    pixmap = QtGui.QPixmap(16, 16)
    pixmap.fill(QtCore.Qt.GlobalColor.transparent)
    painter = QtGui.QPainter(pixmap)
    painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
    path = QtGui.QPainterPath()
    path.moveTo(8, 1)
    for x, y in ((10, 6), (15, 8), (10, 10), (8, 15), (6, 10), (1, 8), (6, 6)):
        path.lineTo(x, y)
    path.closeSubpath()
    painter.fillPath(path, QtGui.QColor("#b4c6e4"))
    painter.end()
    return QtGui.QIcon(pixmap)


def local_time(value):
    return datetime.fromisoformat(value).astimezone().strftime("%Y-%m-%d %H:%M:%S")


class CheckpointDelegate(QtWidgets.QStyledItemDelegate):
    """Strong fill belongs to the live version, never to transient selection."""
    def paint(self, painter, option, index):
        opt = QtWidgets.QStyleOptionViewItem(option)
        self.initStyleOption(opt, index)
        selected = bool(opt.state & QtWidgets.QStyle.StateFlag.State_Selected)
        current = bool(index.data(CURRENT_ROLE))
        opt.state &= ~(QtWidgets.QStyle.StateFlag.State_Selected | QtWidgets.QStyle.StateFlag.State_HasFocus)
        if current:
            opt.state |= QtWidgets.QStyle.StateFlag.State_Selected | QtWidgets.QStyle.StateFlag.State_Active
            opt.palette.setCurrentColorGroup(QtGui.QPalette.ColorGroup.Active)
            if selected:
                opt.palette.setColor(QtGui.QPalette.ColorRole.Highlight,
                                     opt.palette.color(QtGui.QPalette.ColorRole.Highlight).lighter(125))
            opt.font.setBold(True)
        elif selected:
            tint = opt.palette.color(QtGui.QPalette.ColorRole.Text)
            tint.setAlpha(28)
            opt.backgroundBrush = QtGui.QBrush(tint)
        style = opt.widget.style() if opt.widget else QtWidgets.QApplication.style()
        style.drawControl(QtWidgets.QStyle.ControlElement.CE_ItemViewItem, opt, painter, opt.widget)
        if selected:
            painter.save()
            color = opt.palette.color(QtGui.QPalette.ColorRole.HighlightedText if current
                                      else QtGui.QPalette.ColorRole.Text)
            color = QtGui.QColor("white") if current else color
            color.setAlpha(255 if current else 230)
            pen = QtGui.QPen(color, 2 if current else 1, QtCore.Qt.PenStyle.SolidLine)
            painter.setPen(pen)
            rect = opt.rect.adjusted(1, 1, -1, -1)
            painter.drawLine(rect.topLeft(), rect.topRight())
            painter.drawLine(rect.bottomLeft(), rect.bottomRight())
            if index.column() == 0:
                painter.drawLine(rect.topLeft(), rect.bottomLeft())
            if index.column() == index.model().columnCount() - 1:
                painter.drawLine(rect.topRight(), rect.bottomRight())
            painter.restore()


class Panel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("loom_checkpoints_panel")
        self._closed = False
        self.session = acquire(self)
        self.workspace = self.session.workspace
        try:
            self._build_ui()
            self.refresh_versions()
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(lambda: self._guard(self._refresh_if_changed))
            self.timer.start(500)
            self._key_targets = [self, *self.findChildren(QtWidgets.QWidget)]
            for widget in self._key_targets:
                widget.installEventFilter(self)
        except Exception:
            self.session.release(self)
            raise

    @property
    def busy(self):
        return self.session.turn_owner is not None

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        toolbar = QtWidgets.QHBoxLayout()
        layout.addLayout(toolbar)
        self.checkpoint_button = button("New checkpoint", lambda: self._guard(self.make_checkpoint), toolbar)
        self.capture_button = button("Capture preview", lambda: self._guard(self.capture_active_preview), toolbar)
        self.capture_button.setToolTip("Save the current checkpoint, then capture the viewport. May cook geometry.")
        toolbar.addStretch()
        more = QtWidgets.QToolButton()
        more.setText("⋯")
        more.setPopupMode(QtWidgets.QToolButton.ToolButtonPopupMode.InstantPopup)
        menu = QtWidgets.QMenu(more)
        self.show_replaced_check = QtGui.QAction("Show legacy replaced revisions", menu)
        self.show_replaced_check.setCheckable(True)
        self.show_replaced_check.toggled.connect(lambda *_: self.refresh_versions())
        menu.addAction(self.show_replaced_check)
        menu.addAction("Refresh", lambda: self._guard(self.refresh_versions))
        menu.addSeparator()
        self.cleanup_button = menu.addAction("Delete all other checkpoints", lambda: self._guard(self.clean_up))
        self.cleanup_button.setToolTip("Save the current scene into its checkpoint, then delete every other checkpoint. Cannot be undone.")
        more.setMenu(menu)
        toolbar.addWidget(more)
        self.versions = QtWidgets.QTreeWidget()
        self.versions.setHeaderLabels(["Name", "Last updated", "Size"])
        self.versions.setItemDelegate(CheckpointDelegate(self.versions))
        self.versions.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        header = self.versions.header()
        header.setStretchLastSection(False)
        header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Interactive)
        header.setMinimumSectionSize(50)
        self.versions.setColumnWidth(0, 480)
        self.versions.setColumnWidth(1, 185)
        self.versions.setColumnWidth(2, 75)
        self.versions.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.versions.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection)
        self.versions.itemDoubleClicked.connect(lambda *_: self._guard(self.visit_selected))
        self.versions.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.CustomContextMenu)
        self.versions.customContextMenuRequested.connect(self._context_menu)
        layout.addWidget(self.versions, 1)
        self.status = QtWidgets.QLabel("Double-click to switch · F2 to rename · Edits are saved into the current checkpoint before switching")
        self.status.setWordWrap(True)
        self.status.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        layout.addWidget(self.status)
        self.storage_label = QtWidgets.QLabel()
        self.storage_label.setWordWrap(True)
        self.storage_label.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        layout.addWidget(self.storage_label)
        self.error_details_button = QtWidgets.QToolButton()
        self.error_details_button.setText("Error details")
        self.error_details_button.setCheckable(True)
        self.error_details_button.hide()
        layout.addWidget(self.error_details_button)
        self.error_details = QtWidgets.QPlainTextEdit()
        self.error_details.setReadOnly(True)
        self.error_details.setMaximumHeight(100)
        self.error_details.hide()
        self.error_details_button.toggled.connect(self.error_details.setVisible)
        layout.addWidget(self.error_details)

    def eventFilter(self, watched, event):
        # Houdini may claim F2 before a QShortcut sees it. Accept the override
        # only inside this pane; never consume keys from other panes/dialogs.
        if (not self._closed
                and event.type() in (QtCore.QEvent.Type.ShortcutOverride, QtCore.QEvent.Type.KeyPress)
                and isinstance(watched, QtWidgets.QWidget)
                and (watched is self or self.isAncestorOf(watched))
                and QtWidgets.QApplication.activeModalWidget() is None
                and event.modifiers() == QtCore.Qt.KeyboardModifier.NoModifier):
            rename = event.key() == QtCore.Qt.Key.Key_F2
            visit = (event.key() in (QtCore.Qt.Key.Key_Return, QtCore.Qt.Key.Key_Enter)
                     and watched in (self.versions, self.versions.viewport()))
            if rename or visit:
                event.accept()
                if event.type() == QtCore.QEvent.Type.KeyPress and not event.isAutoRepeat():
                    self._guard(self.rename_checkpoint if rename else self.visit_selected)
                return True
        return super().eventFilter(watched, event)

    def _menu(self):
        self._ensure_binding()
        ids = self.selected_ids()
        menu = QtWidgets.QMenu(self)
        if len(ids) == 1:
            menu.addAction("Visit", lambda: self._guard(self.visit_selected)).setEnabled(not self.busy)
            menu.addAction("Rename…", lambda: self._guard(self.rename_checkpoint))
            menu.addAction("Compare with current scene", lambda: self._guard(self.compare)).setEnabled(not self.busy)
            menu.addAction("Replace with current scene…", lambda: self._guard(self.replace_checkpoint)).setEnabled(not self.busy)
            delete = menu.addAction("Delete checkpoint", lambda: self._guard(self.delete_selected))
            delete.setEnabled(not self.busy and ids[0] != self.workspace.active)
            if ids[0] == self.workspace.active:
                delete.setToolTip("Visit another checkpoint before deleting this one.")
            descendants = self.workspace.history.descendants(ids[0])
            delete_children = menu.addAction("Delete descendant checkpoints", lambda: self._guard(self.delete_descendants))
            delete_children.setEnabled(not self.busy and bool(descendants) and self.workspace.active not in descendants)
            delete_children.setToolTip("Delete all children and grandchildren, keeping this checkpoint. "
                                      "Unavailable when any descendant is current.")
        elif len(ids) == 2:
            menu.addAction("Compare checkpoints", lambda: self._guard(self.compare)).setEnabled(not self.busy)
        for extension in registered_actions():
            if len(ids) == extension.selection_count:
                selected = tuple(ids)
                workspace_id = self.workspace.history.id
                def invoke(extension=extension, ids=selected, workspace_id=workspace_id):
                    self._ensure_binding()
                    if self.workspace.history.id != workspace_id:
                        raise RuntimeError("Scene changed; reopen the checkpoint menu.")
                    extension.callback(self.workspace, list(ids), self)
                    self.refresh_versions()
                menu.addAction(extension.label, lambda invoke=invoke: self._guard(invoke)).setEnabled(not self.busy)
        return menu

    def _context_menu(self, position):
        def show():
            item = self.versions.itemAt(position)
            if item is not None and not item.isSelected():
                self.versions.clearSelection()
                item.setSelected(True)
            menu = self._menu()
            try:
                menu.exec(self.versions.viewport().mapToGlobal(position))
            finally:
                menu.deleteLater()
        self._guard(show)

    def _ensure_binding(self):
        if self.busy:
            from .scene import _hou, WORKSPACE_KEY
            if _hou().node("/").userData(WORKSPACE_KEY) != self.workspace.history.id:
                raise RuntimeError("Stop the active Loom turn before switching workspace.")
        else:
            self.workspace.refresh_binding()

    def _require_idle(self):
        self.session.require_idle()
        self._ensure_binding()

    def _refresh_if_changed(self):
        self.workspace.process_pending_save()
        self._ensure_binding()
        signature = (self.workspace.history.id, self.workspace.active,
                     repr(self.workspace.history.index), self.busy, self.workspace.save_warning)
        if signature != getattr(self, "_signature", None):
            self._signature = signature
            self.refresh_versions()
        if self.workspace.save_warning:
            self.status.setText(self.workspace.save_warning)
        self._update_buttons()

    def _update_buttons(self):
        self.checkpoint_button.setEnabled(not self.busy)
        self.capture_button.setEnabled(not self.busy and bool(self.workspace.active))
        self.cleanup_button.setEnabled(not self.busy)

    def _guard(self, callback):
        try:
            return callback()
        except Exception as exc:
            self.report_error(exc)

    def report_error(self, exc):
        self.status.setText(str(exc))
        self.error_details.setPlainText(traceback.format_exc())
        self.error_details_button.show()

    def refresh_versions(self, select_id=None):
        selected = {select_id} if select_id is not None else set(self.selected_ids())
        self.versions.clear()
        items = {}
        records = {r["id"]: r for r in self.workspace.history.entries}
        visible = [r for r in records.values() if not r.get("replaced_by")
                   or self.show_replaced_check.isChecked() or r["id"] == self.workspace.active]
        for record in visible:
            current = record["id"] == self.workspace.active
            label = record["label"]
            if record.get("replaced_by"):
                label += " (legacy revision)"
            updated = record.get("updated", record["created"])
            item = QtWidgets.QTreeWidgetItem([label, local_time(updated),
                                             f"{record.get('bytes', 0) / 1048576:.1f} MB"])
            item.setData(0, QtCore.Qt.ItemDataRole.UserRole, record["id"])
            if record.get("created_by") == "ai":
                item.setIcon(0, ai_icon())
            for column in range(3):
                item.setData(column, CURRENT_ROLE, current)
                if current:
                    font = item.font(column)
                    font.setBold(True)
                    item.setFont(column, font)
                item.setToolTip(column, "Created: " + local_time(record["created"])
                                + "\nLast updated: " + local_time(updated)
                                + ("\nCreated by Loom (AI)" if record.get("created_by") == "ai" else "")
                                + "\nSource: " + record["source_hip"] + "\n" + record["id"])
            items[record["id"]] = item
        for record in visible:
            item = items[record["id"]]
            parent_id = record.get("parent")
            seen = {record["id"]}
            while parent_id in records and parent_id not in items and parent_id not in seen:
                seen.add(parent_id)
                parent_id = records[parent_id].get("parent")
            parent = items.get(parent_id)
            if parent is not None and parent_id not in seen:
                parent.addChild(item)
            else:
                self.versions.addTopLevelItem(item)
            item.setSelected(record["id"] in selected)
        self.versions.expandAll()
        if select_id in items:
            self.versions.setCurrentItem(items[select_id], 0,
                QtCore.QItemSelectionModel.SelectionFlag.ClearAndSelect | QtCore.QItemSelectionModel.SelectionFlag.Rows)
            self.versions.scrollToItem(items[select_id])
        total = sum(r.get("bytes", 0) for r in records.values())
        self.storage_label.setText(f"{len(visible)} checkpoints · {total / 1048576:.1f} MB")
        self.storage_label.setToolTip(str(self.workspace.history.root))
        self._update_buttons()

    def selected_ids(self):
        return [i.data(0, QtCore.Qt.ItemDataRole.UserRole) for i in self.versions.selectedItems()]

    def make_checkpoint(self):
        self._require_idle()
        record = self.workspace.checkpoint()
        self.refresh_versions(select_id=record["id"])
        self.status.setText("New checkpoint created. Working HIP filename unchanged.")

    def visit_selected(self):
        self._require_idle()
        selected = self.selected_ids()
        if len(selected) != 1:
            raise ValueError("Select exactly one checkpoint to visit")
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.CursorShape.WaitCursor)
        try:
            result = self.workspace.visit(selected[0])
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        self.refresh_versions()
        self.status.setText(("Already at this checkpoint." if result["unchanged"] else
                             "Checkpoint loaded. Changes were saved into the checkpoint you left.")
                            + " " + self.workspace.last_warning)

    def compare(self):
        self._ensure_binding()
        selected = self.selected_ids()
        if not 1 <= len(selected) <= 2:
            raise ValueError("Select one checkpoint to compare with live state, or two checkpoints")
        CompareDialog(self.workspace, selected[0], selected[1] if len(selected) == 2 else None, self).exec()

    def rename_checkpoint(self):
        self._ensure_binding()
        selected = self.selected_ids()
        if len(selected) != 1:
            raise ValueError("Select one checkpoint")
        record = self.workspace.history.get(selected[0])
        label, accepted = prompt_text(self, "Rename checkpoint", "Label:", text=record["label"])
        if accepted:
            self.workspace.history.rename(selected[0], label)
            self.refresh_versions()

    def replace_checkpoint(self):
        self._require_idle()
        selected = self.selected_ids()
        if len(selected) != 1:
            raise ValueError("Select exactly one checkpoint to replace")
        if QtWidgets.QMessageBox.question(self, "Replace checkpoint?",
                "Overwrite the selected checkpoint with the current scene? Its previous contents will not be kept.",
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.No) != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        self.workspace.replace(selected[0])
        self.refresh_versions()
        self.status.setText("Selected checkpoint updated and made current.")

    def capture_active_preview(self):
        self._require_idle()
        self.workspace.capture_current_preview()
        self.refresh_versions()
        self.status.setText("Current checkpoint saved and viewport preview captured.")

    def delete_selected(self):
        self._require_idle()
        selected = self.selected_ids()
        if len(selected) != 1:
            raise ValueError("Select one checkpoint to delete")
        result = self.workspace.delete(selected[0])
        self.refresh_versions()
        self._show_file_errors(result, "Checkpoint deleted. Its children were moved up one level.")

    def delete_descendants(self):
        self._require_idle()
        selected = self.selected_ids()
        if len(selected) != 1:
            raise ValueError("Select one parent checkpoint")
        result = self.workspace.delete_descendants(selected[0])
        self.refresh_versions()
        self._show_file_errors(result, "Descendant checkpoints deleted. Parent checkpoint kept.")

    def clean_up(self):
        self._require_idle()
        result = self.workspace.cleanup()
        self.refresh_versions()
        self._show_file_errors(result, "Current checkpoint saved. All other checkpoints deleted.")

    def _show_file_errors(self, result, success):
        if result["file_errors"]:
            self.status.setText(success + " Some files could not be removed; use Delete all other checkpoints to retry.")
            self.error_details.setPlainText(pretty(result["file_errors"]))
            self.error_details_button.show()
        else:
            self.status.setText(success)

    def shutdown(self):
        if not self._closed:
            self._closed = True
            self.timer.stop()
            from shiboken6 import isValid
            for widget in self._key_targets:
                if isValid(widget):
                    widget.removeEventFilter(self)
            self.session.release(self)

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)

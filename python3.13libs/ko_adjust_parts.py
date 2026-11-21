"""Compact, node-aware Parts panel for KoAdjust."""
import hou
from hutil.Qt import QtCore, QtGui, QtWidgets

import ko_adjust as adjust

INTERFACE = 'ko_adjust_parts'


class PartsModel(QtCore.QAbstractTableModel):
    def __init__(self, parent):
        super().__init__(parent)
        self.names = []
        self.hidden = self.editing = set()
        self.icons = [hou.qt.Icon('SCENEGRAPH_visible_on', 16, 16),
                      hou.qt.Icon('SCENEGRAPH_visible_off', 16, 16)]

    def rowCount(self, parent=QtCore.QModelIndex()):
        return 0 if parent.isValid() else len(self.names)

    def columnCount(self, parent=QtCore.QModelIndex()):
        return 0 if parent.isValid() else 2

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid():
            return None
        name = self.names[index.row()]
        label = name or '(unnamed part)'
        eye = index.column() == 1
        hidden = name in self.hidden
        if role == QtCore.Qt.DisplayRole and not eye:
            return label
        if role == QtCore.Qt.DecorationRole and eye:
            return self.icons[hidden]
        if role in (QtCore.Qt.ToolTipRole, QtCore.Qt.AccessibleTextRole):
            return ('Show ' if hidden else 'Hide ') + label if eye else label
        if role == QtCore.Qt.FontRole and not eye and name in self.editing:
            font = QtGui.QFont(self.parent().font())
            font.setBold(True)
            return font
        return None

    def update_parts(self, names, hidden, editing):
        if self.names != names:
            self.beginResetModel()
            self.names, self.hidden, self.editing = names, hidden, editing
            self.endResetModel()
            return
        changed = (self.hidden ^ hidden) | (self.editing ^ editing)
        self.hidden, self.editing = hidden, editing
        if changed and names:
            self.dataChanged.emit(self.index(0, 0), self.index(len(names)-1, 1), [])


class PartsTable(QtWidgets.QTableView):
    toggleRequested = QtCore.Signal(QtCore.QModelIndex)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Space:
            self.toggleRequested.emit(self.currentIndex())
            event.accept()
        else:
            super().keyPressEvent(event)


class PartsPanel(QtWidgets.QWidget):
    _events = (hou.nodeEventType.InputRewired, hou.nodeEventType.InputDataChanged,
               hou.nodeEventType.ParmTupleChanged, hou.nodeEventType.BeingDeleted)
    _part_parms = {'part_source', 'part_attribute', 'uv_attribute', 'view_hidden', 'editing_parts_data'}

    def __init__(self):
        super().__init__()
        self.node = None
        self._tab_bar = None
        self.needs_names = True
        self._callback = self._node_event
        self.timer = QtCore.QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.setInterval(0)
        self.timer.timeout.connect(self.refresh)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(5)
        bar = QtWidgets.QHBoxLayout()
        self.filter = QtWidgets.QLineEdit(placeholderText='Filter parts…')
        self.filter.setClearButtonEnabled(True)
        self.show_all = QtWidgets.QPushButton('Show All')
        self.show_all.clicked.connect(self._show_all)
        bar.addWidget(self.filter, 1)
        bar.addWidget(self.show_all)
        layout.addLayout(bar)

        self.model = PartsModel(self)
        self.proxy = QtCore.QSortFilterProxyModel(self)
        self.proxy.setSourceModel(self.model)
        self.proxy.setFilterCaseSensitivity(QtCore.Qt.CaseInsensitive)
        self.filter.textChanged.connect(self.proxy.setFilterFixedString)
        self.table = PartsTable()
        self.table.setModel(self.proxy)
        self.table.setShowGrid(False)
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        vertical = self.table.verticalHeader()
        vertical.hide()
        vertical.setSectionResizeMode(QtWidgets.QHeaderView.Fixed)
        vertical.setMinimumSectionSize(18)
        vertical.setDefaultSectionSize(max(20, self.fontMetrics().height()+4))
        header = self.table.horizontalHeader()
        header.hide()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Fixed)
        header.resizeSection(1, 28)
        self.table.clicked.connect(self._clicked)
        self.table.toggleRequested.connect(self._toggle)
        layout.addWidget(self.table, 1)
        self.status = QtWidgets.QLabel()
        layout.addWidget(self.status)
        self.refresh()

    def hide_chrome(self, pane):
        if pane is None:
            return
        pane.showNetworkControls(False)
        pane.showToolbar(False)
        holder=pane.pane()
        if holder is not None and pane.isCurrentTab() and self._tab_bar is None:
            self._tab_bar=(holder,holder.isShowingPaneTabs())
            holder.showPaneTabs(False)

    def restore_tab_bar(self):
        saved,self._tab_bar=self._tab_bar,None
        if saved is not None:
            holder,was_shown=saved
            try:
                holder.showPaneTabs(was_shown)
            except hou.ObjectWasDeleted:
                pass

    def bind(self, node):
        try:
            while node is not None and node.type().name() != adjust.TYPE:
                node = node.parent()
        except (AttributeError, hou.ObjectWasDeleted):
            node = None
        if node == self.node:
            self.refresh(force=True)
            return
        self.shutdown()
        self.node = node
        if node is not None:
            node.addEventCallback(self._events, self._callback)
        self.refresh(force=True)

    def shutdown(self):
        self.timer.stop()
        if self.node is not None:
            try:
                self.node.removeEventCallback(self._events, self._callback)
            except (hou.ObjectWasDeleted, hou.OperationFailed):
                pass
        self.node = None
        self.needs_names = True

    def _node_event(self, event_type, parm_tuple=None, **kwargs):
        if event_type == hou.nodeEventType.BeingDeleted:
            self.node = None
        elif event_type == hou.nodeEventType.ParmTupleChanged and parm_tuple is not None:
            name = parm_tuple.name()
            if name not in self._part_parms:
                return
            if name not in ('view_hidden', 'editing_parts_data'):
                self.needs_names = True
        else:
            self.needs_names = True
        self.timer.start()

    def refresh(self, force=False):
        try:
            if self.node is None:
                names, hidden, editing = [], set(), set()
            else:
                names = adjust.parts(self.node) if force or self.needs_names else self.model.names
                hidden = adjust.hidden_parts(self.node)
                editing = adjust.editing_parts(self.node)
        except (hou.ObjectWasDeleted, hou.OperationFailed):
            names, hidden, editing = [], set(), set()
        self.needs_names = False
        self.model.update_parts(names, hidden, editing)
        self.show_all.setEnabled(bool(hidden))
        shown = len(names) - sum(name in hidden for name in names)
        self.status.setText('Select a KoAdjust SOP' if self.node is None else
                            'No parts on this input' if not names else f'{shown} of {len(names)} visible')

    def _clicked(self, index):
        if index.column() == 1:
            self._toggle(index)

    def _toggle(self, index):
        source = self.proxy.mapToSource(index)
        if self.node is not None and source.isValid():
            name = self.model.names[source.row()]
            try:
                adjust.set_hidden_parts(self.node, adjust.hidden_parts(self.node) ^ {name})
            except hou.ObjectWasDeleted:
                self.bind(None)
            self.refresh()

    def _show_all(self):
        if self.node is not None:
            adjust.set_hidden_parts(self.node, set())
            self.refresh()


def open_panel(node):
    pane = next((candidate for candidate in hou.ui.paneTabs()
                 if candidate.type() == hou.paneTabType.PythonPanel and candidate.activeInterface()
                 and candidate.activeInterface().name() == INTERFACE), None)
    if pane is None:
        pane = hou.ui.curDesktop().createFloatingPaneTab(hou.paneTabType.PythonPanel,
            size=(290, 360), python_panel_interface=INTERFACE, immediate=True)
    pane.setIsCurrentTab()
    pane.setCurrentNode(node)
    widget = pane.activeInterfaceRootWidget()
    if widget is not None:
        widget.hide_chrome(pane)
        widget.bind(node)
        widget.window().raise_()
    return pane

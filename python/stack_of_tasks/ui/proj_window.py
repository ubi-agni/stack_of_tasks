from PyQt5.QtCore import QEvent, Qt, pyqtSignal
from PyQt5.QtGui import QActionEvent, QKeySequence
from PyQt5.QtWidgets import (
    QAction,
    QHBoxLayout,
    QHeaderView,
    QMainWindow,
    QTableWidget,
    QTableWidgetItem,
    QToolButton,
    QVBoxLayout,
    QWidget,
    qApp,
)

from .generated.MainWindow import Ui_MainWindow


class Ui_Project_Window(QMainWindow):

    open_project = pyqtSignal(int)
    open_project_from_file = pyqtSignal()
    new_project = pyqtSignal()

    def __init__(self):
        super().__init__()

        self._last_sel_idx = 0

        self._central = QWidget()
        self.setCentralWidget(self._central)

        self._l = QVBoxLayout()
        self._central.setLayout(self._l)

        # list of recent projects
        self._proj_list = QTableWidget()
        self._l.addWidget(self._proj_list)

        self._proj_list.itemSelectionChanged.connect(self._sel_changed)
        self._proj_list.cellDoubleClicked.connect(self._open_from_list)

        self._proj_list.setColumnCount(2)
        self._proj_list.setHorizontalHeaderLabels(["Name", "Date"])
        self._proj_list.verticalHeader().hide()
        self._proj_list.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)

        self._proj_list.setSelectionBehavior(QTableWidget.SelectRows)
        self._proj_list.setEditTriggers(QTableWidget.NoEditTriggers)
        self._proj_list.setFocusPolicy(Qt.NoFocus)
        self._proj_list.setAlternatingRowColors(True)

        item_proto = QTableWidgetItem()
        item_proto.setFlags(item_proto.flags() ^ Qt.ItemIsEditable)
        self._proj_list.setItemPrototype(item_proto)

        # actions

        _open_recent_action = QAction("Open", self)
        _open_recent_action.triggered.connect(self._open_recent_action_triggered)

        self._open_recent_btn = QToolButton()
        self._open_recent_btn.setDefaultAction(_open_recent_action)

        _open_file_action = QAction("Open file")
        _open_file_action.setShortcut(Qt.Key_O + Qt.CTRL)
        _open_file_action.triggered.connect(self._open_file_action_triggered)

        self._open_file_btn = QToolButton()
        self._open_file_btn.setDefaultAction(_open_file_action)

        _new_action = QAction("New")
        _new_action.setShortcut(Qt.Key_N + Qt.CTRL)
        _new_action.triggered.connect(self._new_action_triggered)

        self._new_action_btn = QToolButton()
        self._new_action_btn.setDefaultAction(_new_action)

        _bg = QHBoxLayout()
        self._l.addLayout(_bg)

        _bg.addWidget(self._open_recent_btn)
        _bg.addWidget(self._open_file_btn)
        _bg.addWidget(self._new_action_btn)
        _bg.addStretch(1)

        self.resize(800, 500)
        self.show()

    def _sel_changed(self):
        sel_items = self._proj_list.selectedItems()
        if len(sel_items) > 1:
            self._last_sel_item = sel_items[0].row()
        else:
            self._proj_list.selectRow(self._last_sel_idx)

    def _table_item(self, text: str) -> QTableWidgetItem:
        item = self._proj_list.itemPrototype().clone()
        item.setText(text)
        return item

    def set_last_items(self, l: list):
        self._proj_list.setRowCount(len(l))

        for i, item in enumerate(l):
            self._proj_list.setItem(i, 0, self._table_item(item["name"]))
            self._proj_list.setItem(i, 1, self._table_item(item["date"]))

        self._proj_list.selectRow(0)

    def _new_action_triggered(self):
        self.new_project.emit()

    def _open_file_action_triggered(self):
        self.open_project_from_file.emit()

    def _open_recent_action_triggered(self):
        row = self._proj_list.selectedIndexes()[0].row()
        self.open_project.emit(row)

    def _open_from_list(self, row, _):
        self.open_project.emit(row)

    def show(self) -> None:
        return super().show()

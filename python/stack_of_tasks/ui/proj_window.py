from sys import version_info as vi

import PyQt5
from PyQt5 import Qt, QtCore, QtWidgets
from PyQt5.QtGui import QPainter

from .generated.MainWindow import Ui_MainWindow


class Ui_Project_Window(QtWidgets.QMainWindow):

    open_project = QtCore.pyqtSignal(int)
    open_project_from_file = QtCore.pyqtSignal()
    new_project = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()

        self._last_sel_idx = 0

        self._central = QtWidgets.QWidget()
        self.setCentralWidget(self._central)

        self._l = QtWidgets.QGridLayout()
        self._central.setLayout(self._l)

        self.last_projects = QtWidgets.QTableWidget()
        self.last_projects.setColumnCount(2)
        self.last_projects.setHorizontalHeaderLabels(["Name", "Date"])
        self.last_projects.verticalHeader().hide()
        self.last_projects.horizontalHeader().setSectionResizeMode(
            0, QtWidgets.QHeaderView.Stretch
        )

        self.last_projects.setSelectionBehavior(QtWidgets.QTableWidget.SelectRows)
        self.last_projects.setEditTriggers(QtWidgets.QTableWidget.NoEditTriggers)
        self.last_projects.setFocusPolicy(QtCore.Qt.NoFocus)
        self.last_projects.setAlternatingRowColors(True)
        item_proto = QtWidgets.QTableWidgetItem()
        item_proto.setFlags(item_proto.flags() ^ QtCore.Qt.ItemIsEditable)

        self.last_projects.setItemPrototype(item_proto)
        self._l.addWidget(self.last_projects, 0, 0, 1, 2)

        self.last_projects.itemSelectionChanged.connect(self._sel_changed)

        self.last_projects.cellDoubleClicked.connect(self._open_from_list)

        self._open_button = QtWidgets.QPushButton("Open Project")
        self._open_button.clicked.connect(self._open_project)

        self._l.addWidget(self._open_button, 1, 1)

        self._new_pro_button = QtWidgets.QPushButton("New")
        self._new_pro_button.clicked.connect(self._new_project)

        self._new_pro_button.setSizePolicy(
            QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred
        )
        self._l.addWidget(self._new_pro_button, 0, 2)

        self.resize(800, 500)
        self.show()

    def _sel_changed(self):
        sel_items = self.last_projects.selectedItems()
        if len(sel_items) > 1:
            self._last_sel_item = sel_items[0].row()
        else:
            self.last_projects.selectRow(self._last_sel_idx)

    def _table_item(self, text: str) -> QtWidgets.QTableWidgetItem:
        item = self.last_projects.itemPrototype().clone()
        item.setText(text)
        return item

    def set_last_items(self, l: list):
        self.last_projects.setRowCount(len(l))

        for i, item in enumerate(l):
            self.last_projects.setItem(i, 0, self._table_item(item["name"]))
            self.last_projects.setItem(i, 1, self._table_item(item["date"]))

        self.last_projects.selectRow(0)

    def _new_project(self):
        self.new_project.emit()

    def _open_project(self):
        self.open_project_from_file.emit()

    def _open_from_list(self, row, _):
        self.open_project.emit(row)

    def show(self) -> None:
        self.move(QtWidgets.qApp.desktop().rect().center() - self.rect().center())
        return super().show()

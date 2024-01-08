from sys import version_info as vi

import PyQt5
from PyQt5 import Qt, QtCore, QtWidgets

from .generated.MainWindow import Ui_MainWindow


class Ui_Project_Window(QtWidgets.QMainWindow):

    open_project = QtCore.pyqtSignal(int)

    def __init__(self):
        super().__init__()

        self._central = QtWidgets.QWidget()
        self.setCentralWidget(self._central)

        self._l = QtWidgets.QGridLayout()
        self._central.setLayout(self._l)

        self.last_projects = QtWidgets.QListWidget()
        self._l.addWidget(self.last_projects, 0, 0, 1, 2)

        self._open_button = QtWidgets.QPushButton("Open")
        self._open_button.clicked.connect(self._open_project)
        self._l.addWidget(self._open_button, 1, 1)

        self._new_pro_button = QtWidgets.QPushButton("Add")

        self._new_pro_button.setSizePolicy(
            QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred
        )
        self._l.addWidget(self._new_pro_button, 0, 2)

        self.show()

    def set_last_items(self, l: list):
        for i in l:
            self.last_projects.addItem(i["name"])

    def new_project(self):
        pass

    def _open_project(self):
        i = self.last_projects.selectedIndexes()[0].row()
        print(i)
        self.open_project.emit(i)

    def show(self) -> None:
        self.move(QtWidgets.qApp.desktop().rect().center() - self.rect().center())
        return super().show()

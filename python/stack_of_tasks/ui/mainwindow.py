from distutils import version
from sys import version_info as vi

from PyQt5 import Qt, QtWidgets
from PyQt5.QtWidgets import QAction, QPushButton

from .generated.MainWindow import Ui_MainWindow


class Ui(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, ref_model, task_hierarchy):
        super(Ui, self).__init__()
        self.setupUi(self)

        infoMenu = self.menu_bar.addMenu("Info")

        info1 = QtWidgets.QWidgetAction(infoMenu)
        infoLabel1 = QtWidgets.QLabel(f"PyQt {Qt.PYQT_VERSION_STR}")
        infoLabel1.setMargin(10)
        info1.setDefaultWidget(infoLabel1)

        info2 = QtWidgets.QWidgetAction(infoMenu)
        infoLabel2 = QtWidgets.QLabel(f"Qt {Qt.QT_VERSION_STR}")
        infoLabel2.setMargin(10)
        info2.setDefaultWidget(infoLabel2)

        info3 = QtWidgets.QWidgetAction(infoMenu)
        infoLabel3 = QtWidgets.QLabel(f"Python {vi.major}.{vi.minor}.{vi.micro}")
        infoLabel3.setMargin(10)
        info3.setDefaultWidget(infoLabel3)

        infoMenu.addAction(info1)
        infoMenu.addAction(info2)
        infoMenu.addAction(info3)

        self.setCentralWidget(self.tab_widget)
        self.tab_widget.hierarchy.set_ref_model(ref_model)
        self.tab_widget.hierarchy.model.set_hierarchy(task_hierarchy)
        self.tab_widget.refs.set_model(ref_model)

        self.run_Button = QPushButton("Start")
        self.status_bar.addPermanentWidget(self.run_Button)
        self.show()

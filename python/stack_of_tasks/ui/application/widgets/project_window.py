from sys import version_info as vi

from PyQt5 import Qt, QtWidgets
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QMainWindow, QMenuBar, QPushButton, QStatusBar, QTabWidget

from stack_of_tasks.ui.tabs.hierarchy import HierarchyTabWidget
from stack_of_tasks.ui.tabs.marker import MarkerTabWidget
from stack_of_tasks.ui.tabs.refs import RefFramesTabWidget
from stack_of_tasks.ui.tabs.solver_settings import SettingsTabWidget


class ProjectUI(QMainWindow):

    open_file_signal = pyqtSignal()
    new_signal = pyqtSignal()
    save_signal = pyqtSignal()
    save_as_signal = pyqtSignal()

    def __init__(self):
        super(ProjectUI, self).__init__()

        self.menu_bar = QMenuBar()
        self.menu_bar.setObjectName("menu_bar")

        self.setMenuBar(self.menu_bar)
        self.status_bar = QStatusBar()
        self.status_bar.setObjectName("status_bar")
        self.setStatusBar(self.status_bar)

        fileMenu = self.menu_bar.addMenu("File")

        self.new_action = fileMenu.addAction("New")
        self.new_action.triggered.connect(self.new_signal)

        self.load_action = fileMenu.addAction("Open File")
        self.load_action.triggered.connect(self.open_file_signal)

        self.save_action = fileMenu.addAction("Save")
        self.save_action.triggered.connect(self.save_signal)

        self.save_as_action = fileMenu.addAction("Save as")
        self.save_as_action.triggered.connect(self.save_as_signal)

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

        self.tab_widget = QTabWidget()

        self.settings_tab = SettingsTabWidget()
        self.tab_widget.addTab(self.settings_tab, "Settings")

        self.hierarchy_tab = HierarchyTabWidget()
        self.tab_widget.addTab(self.hierarchy_tab, "Hierarchy")

        self.refs_tab = RefFramesTabWidget()
        self.tab_widget.addTab(self.refs_tab, "Refs")

        self.marker_tab = MarkerTabWidget()
        self.tab_widget.addTab(self.marker_tab, "Marker")

        self.tab_widget.setCurrentIndex(1)

        self.setCentralWidget(self.tab_widget)
        self.run_Button = QPushButton("Start")
        self.status_bar.addPermanentWidget(self.run_Button)

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/felix/Documents/uni/Arbeit/wsps/pyOASES/src/stack_of_tasks/python/stack_of_tasks/ui/ui_files//MainWindow.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(609, 471)
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        MainWindow.setFont(font)
        MainWindow.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setTabPosition(QtWidgets.QTabWidget.North)
        self.tabWidget.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.tabWidget.setObjectName("tabWidget")
        self.hierarchy = TaskHierarchy()
        self.hierarchy.setObjectName("hierarchy")
        self.tabWidget.addTab(self.hierarchy, "")
        self.targets = Target()
        self.targets.setObjectName("targets")
        self.tabWidget.addTab(self.targets, "")
        self.marker = Marker()
        self.marker.setObjectName("marker")
        self.tabWidget.addTab(self.marker, "")
        self.plot = QtWidgets.QWidget()
        self.plot.setObjectName("plot")
        self.tabWidget.addTab(self.plot, "")
        self.verticalLayout.addWidget(self.tabWidget)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.toggleRun = QtWidgets.QPushButton(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed
        )
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toggleRun.sizePolicy().hasHeightForWidth())
        self.toggleRun.setSizePolicy(sizePolicy)
        self.toggleRun.setObjectName("toggleRun")
        self.horizontalLayout.addWidget(self.toggleRun)
        spacerItem = QtWidgets.QSpacerItem(
            40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum
        )
        self.horizontalLayout.addItem(spacerItem)
        self.verticalLayout.addLayout(self.horizontalLayout)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Task hierarchy Editor"))
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.hierarchy), _translate("MainWindow", "hierarchy")
        )
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.targets), _translate("MainWindow", "Targets")
        )
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.marker), _translate("MainWindow", "Marker")
        )
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.plot), _translate("MainWindow", "Plot")
        )
        self.toggleRun.setText(_translate("MainWindow", "RUN"))


from stack_of_tasks.ui.marker import Marker
from stack_of_tasks.ui.target import Target
from stack_of_tasks.ui.task_hierarchy.task_hierarchy import TaskHierarchy

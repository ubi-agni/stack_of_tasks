# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/felix/Documents/code/hiwi/ws_sot/src/stack_of_tasks/python/stack_of_tasks/ui/ui_files/CentralTabWidgit.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_TabWidget(object):
    def setupUi(self, TabWidget):
        TabWidget.setObjectName("TabWidget")
        TabWidget.resize(429, 239)
        self.solver = SolverTab()
        self.solver.setObjectName("solver")
        TabWidget.addTab(self.solver, "")
        self.hierarchy = HierarchyTab()
        self.hierarchy.setObjectName("hierarchy")
        TabWidget.addTab(self.hierarchy, "")
        self.refs = Ref_Tab()
        self.refs.setObjectName("refs")
        TabWidget.addTab(self.refs, "")

        self.retranslateUi(TabWidget)
        TabWidget.setCurrentIndex(1)

        self.marker = MarkerTab()
        TabWidget.addTab(self.marker, "Marker")

        QtCore.QMetaObject.connectSlotsByName(TabWidget)

    def retranslateUi(self, TabWidget):
        _translate = QtCore.QCoreApplication.translate
        TabWidget.setWindowTitle(_translate("TabWidget", "TabWidget"))
        TabWidget.setTabText(
            TabWidget.indexOf(self.solver), _translate("TabWidget", "Solver")
        )
        TabWidget.setTabText(
            TabWidget.indexOf(self.hierarchy), _translate("TabWidget", "Hierarchy")
        )
        TabWidget.setTabText(TabWidget.indexOf(self.refs), _translate("TabWidget", "Refs"))


from stack_of_tasks.ui.hierarchy_tab.tab import HierarchyTab
from stack_of_tasks.ui.marker_tab.tab import MarkerTab
from stack_of_tasks.ui.ref_tab.tab import Ref_Tab
from stack_of_tasks.ui.solver_tab.tab import SolverTab

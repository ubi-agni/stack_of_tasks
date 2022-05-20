# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './ui/res/TaskHirachy.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_TaskHirachy(object):
    def setupUi(self, TaskHirachy):
        TaskHirachy.setObjectName("TaskHirachy")
        TaskHirachy.resize(845, 505)
        self.horizontalLayout = QtWidgets.QHBoxLayout(TaskHirachy)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.hirachy_scroll = QtWidgets.QScrollArea(TaskHirachy)
        self.hirachy_scroll.setWidgetResizable(True)
        self.hirachy_scroll.setObjectName("hirachy_scroll")
        self.hirachy = QtWidgets.QWidget()
        self.hirachy.setGeometry(QtCore.QRect(0, 0, 409, 485))
        self.hirachy.setObjectName("hirachy")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.hirachy)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.hirachy_scroll.setWidget(self.hirachy)
        self.horizontalLayout.addWidget(self.hirachy_scroll)
        self.tasks = QtWidgets.QListWidget(TaskHirachy)
        self.tasks.setDragEnabled(True)
        self.tasks.setDragDropOverwriteMode(False)
        self.tasks.setDragDropMode(QtWidgets.QAbstractItemView.DragOnly)
        self.tasks.setViewMode(QtWidgets.QListView.ListMode)
        self.tasks.setObjectName("tasks")
        self.horizontalLayout.addWidget(self.tasks)

        self.retranslateUi(TaskHirachy)
        QtCore.QMetaObject.connectSlotsByName(TaskHirachy)

    def retranslateUi(self, TaskHirachy):
        _translate = QtCore.QCoreApplication.translate
        TaskHirachy.setWindowTitle(_translate("TaskHirachy", "Form"))

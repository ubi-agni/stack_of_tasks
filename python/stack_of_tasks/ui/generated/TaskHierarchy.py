# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/felix/Documents/uni/Arbeit/wsps/pyOASES/src/stack_of_tasks/python/stack_of_tasks/ui/ui_files//TaskHierarchy.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_TaskHierarchy(object):
    def setupUi(self, TaskHierarchy):
        TaskHierarchy.setObjectName("TaskHierarchy")
        TaskHierarchy.resize(845, 505)
        self.horizontalLayout = QtWidgets.QHBoxLayout(TaskHierarchy)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.hierarchy_scroll = QtWidgets.QScrollArea(TaskHierarchy)
        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding
        )
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.hierarchy_scroll.sizePolicy().hasHeightForWidth())
        self.hierarchy_scroll.setSizePolicy(sizePolicy)
        self.hierarchy_scroll.setWidgetResizable(True)
        self.hierarchy_scroll.setObjectName("hierarchy_scroll")
        self.hierarchy = QtWidgets.QWidget()
        self.hierarchy.setGeometry(QtCore.QRect(0, 0, 563, 485))
        self.hierarchy.setObjectName("hierarchy")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.hierarchy)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.hierarchy_scroll.setWidget(self.hierarchy)
        self.horizontalLayout.addWidget(self.hierarchy_scroll)
        self.tasks = QtWidgets.QListWidget(TaskHierarchy)
        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding
        )
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tasks.sizePolicy().hasHeightForWidth())
        self.tasks.setSizePolicy(sizePolicy)
        self.tasks.setDragEnabled(True)
        self.tasks.setDragDropOverwriteMode(False)
        self.tasks.setDragDropMode(QtWidgets.QAbstractItemView.DragOnly)
        self.tasks.setViewMode(QtWidgets.QListView.ListMode)
        self.tasks.setObjectName("tasks")
        self.horizontalLayout.addWidget(self.tasks)

        self.retranslateUi(TaskHierarchy)
        QtCore.QMetaObject.connectSlotsByName(TaskHierarchy)

    def retranslateUi(self, TaskHierarchy):
        _translate = QtCore.QCoreApplication.translate
        TaskHierarchy.setWindowTitle(_translate("TaskHierarchy", "Form"))

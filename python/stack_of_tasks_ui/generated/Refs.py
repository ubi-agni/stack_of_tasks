# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/felix/Documents/code/hiwi/ws_sot/src/stack_of_tasks/python/stack_of_tasks/ui/ui_files/Refs.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Refs(object):
    def setupUi(self, Refs):
        Refs.setObjectName("Refs")
        Refs.resize(975, 649)
        self.horizontalLayout = QtWidgets.QHBoxLayout(Refs)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.ref_view = QtWidgets.QTreeView(Refs)
        self.ref_view.setObjectName("ref_view")
        self.horizontalLayout.addWidget(self.ref_view)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.edit_group = QtWidgets.QGroupBox(Refs)
        self.edit_group.setObjectName("edit_group")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.edit_group)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.NoneSelected = QtWidgets.QLabel(self.edit_group)
        self.NoneSelected.setEnabled(True)
        self.NoneSelected.setTextFormat(QtCore.Qt.AutoText)
        self.NoneSelected.setAlignment(QtCore.Qt.AlignCenter)
        self.NoneSelected.setObjectName("NoneSelected")
        self.verticalLayout_2.addWidget(self.NoneSelected)
        self.ref_details = Ref_Details(self.edit_group)
        self.ref_details.setObjectName("ref_details")
        self.verticalLayout_2.addWidget(self.ref_details)
        self.verticalLayout_3.addWidget(self.edit_group)
        self.new_ref = QtWidgets.QToolButton(Refs)
        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed
        )
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.new_ref.sizePolicy().hasHeightForWidth())
        self.new_ref.setSizePolicy(sizePolicy)
        self.new_ref.setObjectName("new_ref")
        self.verticalLayout_3.addWidget(self.new_ref)
        self.horizontalLayout.addLayout(self.verticalLayout_3)
        self.actionadd_ref = QtWidgets.QAction(Refs)
        self.actionadd_ref.setObjectName("actionadd_ref")

        self.retranslateUi(Refs)
        QtCore.QMetaObject.connectSlotsByName(Refs)

    def retranslateUi(self, Refs):
        _translate = QtCore.QCoreApplication.translate
        Refs.setWindowTitle(_translate("Refs", "Refs"))
        self.edit_group.setTitle(_translate("Refs", "Edit Ref"))
        self.NoneSelected.setText(_translate("Refs", "No Task Selected"))
        self.new_ref.setText(_translate("Refs", "add ref"))
        self.actionadd_ref.setText(_translate("Refs", "add ref"))
        self.actionadd_ref.setToolTip(_translate("Refs", "Add new (offset) refference frame"))
        self.actionadd_ref.setShortcut(_translate("Refs", "Ctrl+A"))


from stack_of_tasks_ui.ref_tab.ref_details import Ref_Details

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'python/stack_of_tasks/ui/assets/ui_files/Refs.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
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
        self.ref_view = QtWidgets.QListView(Refs)
        self.ref_view.setObjectName("ref_view")
        self.horizontalLayout.addWidget(self.ref_view)
        self.editor_layout = QtWidgets.QVBoxLayout()
        self.editor_layout.setObjectName("editor_layout")
        self.edit_ref = EditorGroupBox(Refs)
        self.edit_ref.setObjectName("edit_ref")
        self.editor_layout.addWidget(self.edit_ref)
        self.addRef = QtWidgets.QPushButton(Refs)
        self.addRef.setObjectName("addRef")
        self.editor_layout.addWidget(self.addRef)
        self.horizontalLayout.addLayout(self.editor_layout)
        self.actionadd_ref = QtWidgets.QAction(Refs)
        self.actionadd_ref.setObjectName("actionadd_ref")

        self.retranslateUi(Refs)
        QtCore.QMetaObject.connectSlotsByName(Refs)

    def retranslateUi(self, Refs):
        _translate = QtCore.QCoreApplication.translate
        Refs.setWindowTitle(_translate("Refs", "Refs"))
        self.edit_ref.setTitle(_translate("Refs", "Edit ref-frame:"))
        self.addRef.setText(_translate("Refs", "add ref-frame"))
        self.actionadd_ref.setText(_translate("Refs", "add ref"))
        self.actionadd_ref.setToolTip(_translate("Refs", "Add new (offset) refference frame"))
        self.actionadd_ref.setShortcut(_translate("Refs", "Ctrl+A"))
from stack_of_tasks.ui.widgets.has_trait_widgets import EditorGroupBox

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/felix/Documents/code/hiwi/ws_sot/src/stack_of_tasks/python/stack_of_tasks/ui/ui_files/TaskDetails.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_TaskDetails(object):
    def setupUi(self, TaskDetails):
        TaskDetails.setObjectName("TaskDetails")
        TaskDetails.resize(262, 334)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(TaskDetails.sizePolicy().hasHeightForWidth())
        TaskDetails.setSizePolicy(sizePolicy)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(TaskDetails)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setObjectName("formLayout")
        self.nameLabel = QtWidgets.QLabel(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.nameLabel.sizePolicy().hasHeightForWidth())
        self.nameLabel.setSizePolicy(sizePolicy)
        self.nameLabel.setObjectName("nameLabel")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.nameLabel)
        self.nameLineEdit = QtWidgets.QLineEdit(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.nameLineEdit.sizePolicy().hasHeightForWidth())
        self.nameLineEdit.setSizePolicy(sizePolicy)
        self.nameLineEdit.setText("")
        self.nameLineEdit.setObjectName("nameLineEdit")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.nameLineEdit)
        self.softnessTypeLabel = QtWidgets.QLabel(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.softnessTypeLabel.sizePolicy().hasHeightForWidth())
        self.softnessTypeLabel.setSizePolicy(sizePolicy)
        self.softnessTypeLabel.setObjectName("softnessTypeLabel")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.softnessTypeLabel)
        self.softnessTypeComboBox = QtWidgets.QComboBox(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.softnessTypeComboBox.sizePolicy().hasHeightForWidth())
        self.softnessTypeComboBox.setSizePolicy(sizePolicy)
        self.softnessTypeComboBox.setObjectName("softnessTypeComboBox")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.softnessTypeComboBox)
        self.refALabel = QtWidgets.QLabel(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.refALabel.sizePolicy().hasHeightForWidth())
        self.refALabel.setSizePolicy(sizePolicy)
        self.refALabel.setObjectName("refALabel")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.refALabel)
        self.refBLabel = QtWidgets.QLabel(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.refBLabel.sizePolicy().hasHeightForWidth())
        self.refBLabel.setSizePolicy(sizePolicy)
        self.refBLabel.setObjectName("refBLabel")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.refBLabel)
        self.refBComboBox = QtWidgets.QComboBox(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.refBComboBox.sizePolicy().hasHeightForWidth())
        self.refBComboBox.setSizePolicy(sizePolicy)
        self.refBComboBox.setObjectName("refBComboBox")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.refBComboBox)
        self.relLabel = QtWidgets.QLabel(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.relLabel.sizePolicy().hasHeightForWidth())
        self.relLabel.setSizePolicy(sizePolicy)
        self.relLabel.setObjectName("relLabel")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.relLabel)
        self.relComboBox = QtWidgets.QComboBox(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.relComboBox.sizePolicy().hasHeightForWidth())
        self.relComboBox.setSizePolicy(sizePolicy)
        self.relComboBox.setObjectName("relComboBox")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.relComboBox)
        self.weightLabel = QtWidgets.QLabel(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.weightLabel.sizePolicy().hasHeightForWidth())
        self.weightLabel.setSizePolicy(sizePolicy)
        self.weightLabel.setObjectName("weightLabel")
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.LabelRole, self.weightLabel)
        self.weightDoubleSpinBox = QtWidgets.QDoubleSpinBox(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.weightDoubleSpinBox.sizePolicy().hasHeightForWidth())
        self.weightDoubleSpinBox.setSizePolicy(sizePolicy)
        self.weightDoubleSpinBox.setProperty("value", 1.0)
        self.weightDoubleSpinBox.setObjectName("weightDoubleSpinBox")
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.weightDoubleSpinBox)
        self.refAComboBox = QtWidgets.QComboBox(TaskDetails)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.refAComboBox.sizePolicy().hasHeightForWidth())
        self.refAComboBox.setSizePolicy(sizePolicy)
        self.refAComboBox.setObjectName("refAComboBox")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.refAComboBox)
        self.verticalLayout_2.addLayout(self.formLayout)

        self.retranslateUi(TaskDetails)
        QtCore.QMetaObject.connectSlotsByName(TaskDetails)

    def retranslateUi(self, TaskDetails):
        _translate = QtCore.QCoreApplication.translate
        TaskDetails.setWindowTitle(_translate("TaskDetails", "Form"))
        self.nameLabel.setText(_translate("TaskDetails", "Name"))
        self.nameLineEdit.setPlaceholderText(_translate("TaskDetails", "task name"))
        self.softnessTypeLabel.setText(_translate("TaskDetails", "Softness type"))
        self.refALabel.setText(_translate("TaskDetails", "RefA"))
        self.refBLabel.setText(_translate("TaskDetails", "Ref B"))
        self.relLabel.setText(_translate("TaskDetails", "Relation"))
        self.weightLabel.setText(_translate("TaskDetails", "Weight"))
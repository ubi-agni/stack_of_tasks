# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/felix/Documents/uni/Arbeit/wsps/pyOASES/src/stack_of_tasks/python/stack_of_tasks/ui/ui_files//Target.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Target(object):
    def setupUi(self, Target):
        Target.setObjectName("Target")
        Target.resize(400, 300)
        self.verticalLayout = QtWidgets.QVBoxLayout(Target)
        self.verticalLayout.setObjectName("verticalLayout")
        self.add_target_group = QtWidgets.QGroupBox(Target)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.add_target_group.sizePolicy().hasHeightForWidth())
        self.add_target_group.setSizePolicy(sizePolicy)
        self.add_target_group.setFlat(True)
        self.add_target_group.setObjectName("add_target_group")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.add_target_group)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.add_target_form = QtWidgets.QFormLayout()
        self.add_target_form.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        self.add_target_form.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldsStayAtSizeHint)
        self.add_target_form.setObjectName("add_target_form")
        self.nameLabel = QtWidgets.QLabel(self.add_target_group)
        self.nameLabel.setObjectName("nameLabel")
        self.add_target_form.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.nameLabel)
        self.nameLineEdit = QtWidgets.QLineEdit(self.add_target_group)
        self.nameLineEdit.setObjectName("nameLineEdit")
        self.add_target_form.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.nameLineEdit)
        self.typeLabel = QtWidgets.QLabel(self.add_target_group)
        self.typeLabel.setObjectName("typeLabel")
        self.add_target_form.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.typeLabel)
        self.targetType = QtWidgets.QComboBox(self.add_target_group)
        self.targetType.setObjectName("targetType")
        self.add_target_form.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.targetType)
        self.verticalLayout_3.addLayout(self.add_target_form)
        self.addButton = QtWidgets.QPushButton(self.add_target_group)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.addButton.sizePolicy().hasHeightForWidth())
        self.addButton.setSizePolicy(sizePolicy)
        self.addButton.setObjectName("addButton")
        self.verticalLayout_3.addWidget(self.addButton)
        self.verticalLayout.addWidget(self.add_target_group)
        self.targets = TargetList(Target)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.targets.sizePolicy().hasHeightForWidth())
        self.targets.setSizePolicy(sizePolicy)
        self.targets.setObjectName("targets")
        self.verticalLayout.addWidget(self.targets)

        self.retranslateUi(Target)
        QtCore.QMetaObject.connectSlotsByName(Target)

    def retranslateUi(self, Target):
        _translate = QtCore.QCoreApplication.translate
        Target.setWindowTitle(_translate("Target", "Form"))
        self.add_target_group.setTitle(_translate("Target", "Add Target"))
        self.nameLabel.setText(_translate("Target", "Name"))
        self.typeLabel.setText(_translate("Target", "Type"))
        self.addButton.setText(_translate("Target", "Add Target"))
from stack_of_tasks.ui.target_list import TargetList

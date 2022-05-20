# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './ui/res/Marker.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Marker(object):
    def setupUi(self, Marker):
        Marker.setObjectName("Marker")
        Marker.resize(653, 447)
        self.horizontalLayout = QtWidgets.QHBoxLayout(Marker)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label = QtWidgets.QLabel(Marker)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setObjectName("label")
        self.verticalLayout_2.addWidget(self.label)
        self.targets = Targets(Marker)
        self.targets.setObjectName("targets")
        self.verticalLayout_2.addWidget(self.targets)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.availableMarker = QtWidgets.QComboBox(Marker)
        self.availableMarker.setObjectName("availableMarker")
        self.verticalLayout.addWidget(self.availableMarker)
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setObjectName("formLayout")
        self.name = QtWidgets.QLabel(Marker)
        self.name.setObjectName("name")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.name)
        self.name_input = QtWidgets.QLineEdit(Marker)
        self.name_input.setObjectName("name_input")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.name_input)
        self.Transform = QtWidgets.QLabel(Marker)
        self.Transform.setObjectName("Transform")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.Transform)
        self.tranformInput = TransformInput(Marker)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tranformInput.sizePolicy().hasHeightForWidth())
        self.tranformInput.setSizePolicy(sizePolicy)
        self.tranformInput.setObjectName("tranformInput")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.tranformInput)
        self.scaleLabel = QtWidgets.QLabel(Marker)
        self.scaleLabel.setObjectName("scaleLabel")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.scaleLabel)
        self.scale = QtWidgets.QDoubleSpinBox(Marker)
        self.scale.setMinimum(0.05)
        self.scale.setMaximum(1.0)
        self.scale.setProperty("value", 0.1)
        self.scale.setObjectName("scale")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.scale)
        self.verticalLayout.addLayout(self.formLayout)
        self.add = QtWidgets.QPushButton(Marker)
        self.add.setObjectName("add")
        self.verticalLayout.addWidget(self.add)
        self.existingMarker = QtWidgets.QListWidget(Marker)
        self.existingMarker.setObjectName("existingMarker")
        self.verticalLayout.addWidget(self.existingMarker)
        self.horizontalLayout.addLayout(self.verticalLayout)

        self.retranslateUi(Marker)
        QtCore.QMetaObject.connectSlotsByName(Marker)
        Marker.setTabOrder(self.availableMarker, self.name_input)
        Marker.setTabOrder(self.name_input, self.add)

    def retranslateUi(self, Marker):
        _translate = QtCore.QCoreApplication.translate
        Marker.setWindowTitle(_translate("Marker", "Form"))
        self.label.setText(_translate("Marker", "Targets:"))
        self.name.setText(_translate("Marker", "Name"))
        self.name_input.setText(_translate("Marker", "Name"))
        self.Transform.setText(_translate("Marker", "Transform"))
        self.scaleLabel.setText(_translate("Marker", "Scale"))
        self.add.setText(_translate("Marker", "Add"))
from stack_of_tasks.ui.targets import Targets
from stack_of_tasks.ui.transforminput import TransformInput

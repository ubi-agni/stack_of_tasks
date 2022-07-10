# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/felix/Documents/uni/Arbeit/wsps/pyOASES/src/stack_of_tasks/python/stack_of_tasks/ui/ui_files//Marker.ui'
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
        self.line_2 = QtWidgets.QFrame(Marker)
        self.line_2.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.horizontalLayout.addWidget(self.line_2)
        self.right_vbox = QtWidgets.QVBoxLayout()
        self.right_vbox.setObjectName("right_vbox")
        self.add_marker_group = QtWidgets.QGroupBox(Marker)
        self.add_marker_group.setFlat(True)
        self.add_marker_group.setObjectName("add_marker_group")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.add_marker_group)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        self.formLayout.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldsStayAtSizeHint)
        self.formLayout.setObjectName("formLayout")
        self.name = QtWidgets.QLabel(self.add_marker_group)
        self.name.setObjectName("name")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.name)
        self.name_input = QtWidgets.QLineEdit(self.add_marker_group)
        self.name_input.setObjectName("name_input")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.name_input)
        self.scaleLabel = QtWidgets.QLabel(self.add_marker_group)
        self.scaleLabel.setObjectName("scaleLabel")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.scaleLabel)
        self.scale = QtWidgets.QDoubleSpinBox(self.add_marker_group)
        self.scale.setMinimum(0.05)
        self.scale.setMaximum(1.0)
        self.scale.setProperty("value", 0.1)
        self.scale.setObjectName("scale")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.scale)
        self.availableMarker = QtWidgets.QComboBox(self.add_marker_group)
        self.availableMarker.setObjectName("availableMarker")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.availableMarker)
        self.label = QtWidgets.QLabel(self.add_marker_group)
        self.label.setObjectName("label")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label)
        self.verticalLayout.addLayout(self.formLayout)
        self.add = QtWidgets.QPushButton(self.add_marker_group)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.add.sizePolicy().hasHeightForWidth())
        self.add.setSizePolicy(sizePolicy)
        self.add.setObjectName("add")
        self.verticalLayout.addWidget(self.add)
        self.right_vbox.addWidget(self.add_marker_group)
        self.existingMarker = QtWidgets.QListWidget(Marker)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.existingMarker.sizePolicy().hasHeightForWidth())
        self.existingMarker.setSizePolicy(sizePolicy)
        self.existingMarker.setObjectName("existingMarker")
        self.right_vbox.addWidget(self.existingMarker)
        self.horizontalLayout.addLayout(self.right_vbox)

        self.retranslateUi(Marker)
        QtCore.QMetaObject.connectSlotsByName(Marker)

    def retranslateUi(self, Marker):
        _translate = QtCore.QCoreApplication.translate
        Marker.setWindowTitle(_translate("Marker", "Form"))
        self.add_marker_group.setTitle(_translate("Marker", "Add Marker"))
        self.name.setText(_translate("Marker", "Name"))
        self.name_input.setText(_translate("Marker", "Name"))
        self.scaleLabel.setText(_translate("Marker", "Scale"))
        self.label.setText(_translate("Marker", "Type"))
        self.add.setText(_translate("Marker", "Add Marker"))

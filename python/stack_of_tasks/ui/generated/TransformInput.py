# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/felix/Documents/uni/Arbeit/wsps/pyOASES/src/stack_of_tasks/python/stack_of_tasks/ui/ui_files//TransformInput.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(317, 113)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Form.sizePolicy().hasHeightForWidth())
        Form.setSizePolicy(sizePolicy)
        self.horizontalLayout = QtWidgets.QHBoxLayout(Form)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.location = QtWidgets.QFormLayout()
        self.location.setFieldGrowthPolicy(QtWidgets.QFormLayout.ExpandingFieldsGrow)
        self.location.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignTop)
        self.location.setObjectName("location")
        self.x_loc = QtWidgets.QLabel(Form)
        self.x_loc.setObjectName("x_loc")
        self.location.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.x_loc)
        self.z_loc = QtWidgets.QLabel(Form)
        self.z_loc.setObjectName("z_loc")
        self.location.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.z_loc)
        self.y_loc = QtWidgets.QLabel(Form)
        self.y_loc.setObjectName("y_loc")
        self.location.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.y_loc)
        self.z_loc_in = QtWidgets.QLineEdit(Form)
        self.z_loc_in.setMaxLength(100)
        self.z_loc_in.setObjectName("z_loc_in")
        self.location.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.z_loc_in)
        self.y_loc_in = QtWidgets.QLineEdit(Form)
        self.y_loc_in.setMaxLength(100)
        self.y_loc_in.setObjectName("y_loc_in")
        self.location.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.y_loc_in)
        self.x_loc_in = QtWidgets.QLineEdit(Form)
        self.x_loc_in.setMaxLength(100)
        self.x_loc_in.setObjectName("x_loc_in")
        self.location.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.x_loc_in)
        self.horizontalLayout.addLayout(self.location)
        self.rotation = QtWidgets.QFormLayout()
        self.rotation.setObjectName("rotation")
        self.x_rot = QtWidgets.QLabel(Form)
        self.x_rot.setObjectName("x_rot")
        self.rotation.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.x_rot)
        self.z_rot = QtWidgets.QLabel(Form)
        self.z_rot.setObjectName("z_rot")
        self.rotation.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.z_rot)
        self.y_rot = QtWidgets.QLabel(Form)
        self.y_rot.setObjectName("y_rot")
        self.rotation.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.y_rot)
        self.z_rot_in = QtWidgets.QLineEdit(Form)
        self.z_rot_in.setMaxLength(100)
        self.z_rot_in.setObjectName("z_rot_in")
        self.rotation.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.z_rot_in)
        self.y_rot_in = QtWidgets.QLineEdit(Form)
        self.y_rot_in.setMaxLength(100)
        self.y_rot_in.setObjectName("y_rot_in")
        self.rotation.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.y_rot_in)
        self.x_rot_in = QtWidgets.QLineEdit(Form)
        self.x_rot_in.setMaxLength(100)
        self.x_rot_in.setObjectName("x_rot_in")
        self.rotation.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.x_rot_in)
        self.horizontalLayout.addLayout(self.rotation)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)
        Form.setTabOrder(self.x_loc_in, self.y_loc_in)
        Form.setTabOrder(self.y_loc_in, self.z_loc_in)
        Form.setTabOrder(self.z_loc_in, self.x_rot_in)
        Form.setTabOrder(self.x_rot_in, self.y_rot_in)
        Form.setTabOrder(self.y_rot_in, self.z_rot_in)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.x_loc.setText(_translate("Form", "x"))
        self.z_loc.setText(_translate("Form", "z"))
        self.y_loc.setText(_translate("Form", "y"))
        self.x_rot.setText(_translate("Form", "r_x"))
        self.z_rot.setText(_translate("Form", "r_z"))
        self.y_rot.setText(_translate("Form", "r_y"))

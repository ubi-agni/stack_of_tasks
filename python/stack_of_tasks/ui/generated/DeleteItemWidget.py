# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './ui/res/DeleteItemWidget.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_hlayout(object):
    def setupUi(self, hlayout):
        hlayout.setObjectName("hlayout")
        hlayout.resize(711, 41)
        self.horizontalLayout = QtWidgets.QHBoxLayout(hlayout)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.text = QtWidgets.QLabel(hlayout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.text.sizePolicy().hasHeightForWidth())
        self.text.setSizePolicy(sizePolicy)
        self.text.setObjectName("text")
        self.horizontalLayout.addWidget(self.text)
        self.delete_2 = QtWidgets.QPushButton(hlayout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.delete_2.sizePolicy().hasHeightForWidth())
        self.delete_2.setSizePolicy(sizePolicy)
        self.delete_2.setMinimumSize(QtCore.QSize(16, 16))
        self.delete_2.setObjectName("delete_2")
        self.horizontalLayout.addWidget(self.delete_2)

        self.retranslateUi(hlayout)
        QtCore.QMetaObject.connectSlotsByName(hlayout)

    def retranslateUi(self, hlayout):
        _translate = QtCore.QCoreApplication.translate
        hlayout.setWindowTitle(_translate("hlayout", "Form"))
        self.text.setText(_translate("hlayout", "TextLabel"))
        self.delete_2.setText(_translate("hlayout", "del"))

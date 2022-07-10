import numpy
from PyQt5 import QtCore, QtGui, QtWidgets

from ui.DataWidgets.FloatInput import FloatInput

from .DataWidgets.MatrixInput import MatrixInput
from .generated.Target import Ui_Target


class PopupView(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(PopupView, self).__init__(parent)
        self.setWindowFlags(Qt.Popup)
        self.move(QtGui.QCursor.pos())
        self.show()


class Target(QtWidgets.QWidget, Ui_Target):

    value_changed = QtCore.pyqtSignal(str, object)

    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)

        self.targetType.addItems(["Float"])
        self.addButton.clicked.connect(lambda _: self.add_button_click())

        self._widgets = {}

    def add_button_click(self):
        self.add_target(self.nameLineEdit.text(), float)

    def set_target_value(self, name, value):
        if name in self._widgets:
            self._widgets[name].setValue(value)
        else:
            self.add_target(name, type(value))
            self._widgets[name].setValue(value)

    def add_target(self, name, targetType, startVal=None, removeable=False):

        tWidget = None
        if targetType is float:
            tWidget = FloatInput()
        elif targetType is numpy.ndarray:
            tWidget = MatrixInput(1, 1)

        tWidget.value_changed.connect(lambda v: self.value_changed.emit(name, v))
        self.targets.add_target_widget(name, tWidget, removeable)
        self._widgets[name] = tWidget

    def remove_target(self):
        pass

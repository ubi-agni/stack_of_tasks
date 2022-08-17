from PyQt5 import QtWidgets

from .generated.TransformInput import Ui_Form


class TransformInput(QtWidgets.QWidget, Ui_Form):
    def __init__(self, parent=None) -> None:
        super().__init__(parent=parent)
        self.setupUi(self)

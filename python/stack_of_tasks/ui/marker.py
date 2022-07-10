from PyQt5 import QtCore, QtGui, QtWidgets

from .DataWidgets.MatrixInput import MatrixInput
from .generated.Marker import Ui_Marker


class PopupView(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(PopupView, self).__init__(parent)
        self.setWindowFlags(Qt.Popup)
        self.move(QtGui.QCursor.pos())
        self.show()


class Marker(QtWidgets.QWidget, Ui_Marker):

    new_marker_signal = QtCore.pyqtSignal(str, dict)

    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)

        self.verticalLayout.setAlignment(QtCore.Qt.AlignTop)
        self.add.clicked.connect(self._create_marker_slot)

        self.translation = MatrixInput(rowNames=["x", "y", "z"])
        self.rotation = MatrixInput(rowNames=["xr", "yr", "zr"])

        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.translation)
        layout.addWidget(self.rotation)
        self.formLayout.addRow("Transform", layout)

    def _create_marker_slot(self):
        markerClass = self.availableMarker.currentText()
        data = {
            "name": self.name_input.text(),
            "scale": self.scale.value(),
            "pos": self.translation.toNumpy(),
            "rot": self.rotation.toNumpy(),
        }

        self.new_marker_signal.emit(markerClass, data)

    def add_marker(self, name):
        self.existingMarker.addItem(name)

    def _delete(self, name):
        pass

    def set_available_marker_classes(self, marker):
        self.availableMarker.clear()
        self.availableMarker.addItems(marker)

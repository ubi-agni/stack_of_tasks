
from PyQt5 import QtWidgets, QtCore, QtGui
from .generated.Marker import Ui_Marker

class PopupView(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(PopupView, self).__init__(parent)
        self.setWindowFlags(Qt.Popup)
        self.move(QtGui.QCursor.pos())
        self.show()


class Marker(QtWidgets.QWidget, Ui_Marker):

    new_marker = QtCore.pyqtSignal(object,str, tuple, tuple, float)

    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)
        self.available_marker_classes = []
        self.verticalLayout.setAlignment(QtCore.Qt.AlignTop)
        self.add.clicked.connect(self._add)

    def _add(self):
        markerClass = self.available_marker_classes[self.availableMarker.currentIndex()]
        name = self.name_input.text()
        scale = self.scale.value()
        xl = float(self.tranformInput.x_loc_in.text())
        yl = float(self.tranformInput.y_loc_in.text())
        zl = float(self.tranformInput.z_loc_in.text())
        xr = float(self.tranformInput.x_rot_in.text())
        yr = float(self.tranformInput.y_rot_in.text())
        zr = float(self.tranformInput.z_rot_in.text())

        self.existingMarker.addItem(self.availableMarker.currentText())
        self.new_marker.emit(markerClass, name, (xl,yl,zl), (xr,yr,zr), scale)
    
    def _delete(self, name):
        pass

    def set_available_marker_classes(self, classes):
        self.availableMarker.clear()
        for x in classes:
            self.available_marker_classes.append(x)
            self.availableMarker.addItem(x.__name__)
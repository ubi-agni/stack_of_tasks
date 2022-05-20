

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QVBoxLayout
from stack_of_tasks.ui.Collapse import Collapse


class Targets(QtWidgets.QScrollArea):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.setWidgetResizable(True)
        self.setBackgroundRole(QtGui.QPalette.Base)
        self.content = QtWidgets.QWidget()
        self.setWidget(self.content)    

        self.content_layout = QtWidgets.QVBoxLayout()
        self.content.setLayout(self.content_layout)
        self.content_layout.setAlignment(QtCore.Qt.AlignTop)  

        self._widgets = {}


    @QtCore.pyqtSlot(str, object)
    def set_target(self, name, value):
        if name in self._widgets:
            self._widgets[name].content.layout().itemAt(0).widget().setText(str(value))
        else:
            s = Collapse(title=name)
            
            l=QVBoxLayout()
            s.content.setLayout(l)
            ll = QtWidgets.QLabel(str(value))
            l.addWidget(ll)
            
            self.content_layout.addWidget(s)
            self._widgets[name]=s

    @QtCore.pyqtSlot(str)
    def remove_target(self, name):
        pass


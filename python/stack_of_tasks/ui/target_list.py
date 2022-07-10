from turtle import shape

import numpy
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QVBoxLayout
from stack_of_tasks.ui.Collapse import CollapseWithDelete

from .DataWidgets.MatrixInput import MatrixInput


class TargetList(QtWidgets.QScrollArea):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.setWidgetResizable(True)
        self.setBackgroundRole(QtGui.QPalette.Base)
        self.content = QtWidgets.QWidget()
        self.setWidget(self.content)

        self.content_layout = QtWidgets.QVBoxLayout()
        self.content.setLayout(self.content_layout)
        self.content_layout.setAlignment(QtCore.Qt.AlignTop)

    def add_target_widget(self, name, targetWidget, removeable=False):
        collapse = CollapseWithDelete(title=name, deleteable=removeable)
        collapse._contentLayout = QVBoxLayout()
        collapse.content.setLayout(collapse._contentLayout)
        collapse._contentLayout.addWidget(targetWidget)

        collapse.delete_self.connect(lambda: self.remove_target_widget(collapse))
        self.content_layout.addWidget(collapse)

    def remove_target_widget(self, widget: CollapseWithDelete):
        self.content_layout.removeWidget(widget)
        name = widget.toggle_button.text()
        widget.deleteLater()

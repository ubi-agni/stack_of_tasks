from __future__ import annotations

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QComboBox

from stack_of_tasks.ui.models import RefFramesModel


class RefComboBox(QComboBox):
    def __init__(self, model: RefFramesModel = None, editable=True, parent=None) -> None:
        super().__init__(parent)
        self.setEditable(editable)
        # TODO: Insertion requires RefFramesModel.insertRows() to be implemented!
        self.setInsertPolicy(QComboBox.NoInsert)
        if model is not None:
            self.setModel(model)

    def focusInEvent(self, e: QtGui.QFocusEvent) -> None:
        self.update_completer_model()  # update completer model when entering
        super().focusInEvent(e)

    @QtCore.pyqtSlot()
    def update_completer_model(self):
        completer = self.completer()
        if completer is not None:
            completer.setModel(self.model().allRefs())

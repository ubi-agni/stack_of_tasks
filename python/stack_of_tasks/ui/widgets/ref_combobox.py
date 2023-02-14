from __future__ import annotations

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QComboBox, QCompleter

from stack_of_tasks.ui.models import RefFramesModel


class RefComboBox(QComboBox):
    def __init__(self, model: RefFramesModel = None, editable=True, parent=None) -> None:
        super().__init__(parent)
        self._completer: QCompleter = None
        self.setEditable(editable)
        if model is not None:
            self.setModel(model)

    def setEditable(self, editable: bool) -> None:
        super().setEditable(editable)
        if editable:
            self._completer = self.completer()
            self._completer.setCompletionMode(QCompleter.PopupCompletion)
            self.setInsertPolicy(QComboBox.NoInsert)  # insertion handled manually

    def focusInEvent(self, e: QtGui.QFocusEvent) -> None:
        self.update_completer_model()  # update completer model when entering
        super().focusInEvent(e)

    @QtCore.pyqtSlot()
    def update_completer_model(self):
        if self._completer is not None:
            self._completer.setModel(self.model().allRefs())

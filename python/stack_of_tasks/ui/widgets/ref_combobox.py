from __future__ import annotations

from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QComboBox, QCompleter

from stack_of_tasks.ref_frame import JointFrame
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
            self.lineEdit().editingFinished.connect(self.onFinishedEditing)

    def focusInEvent(self, e: QtGui.QFocusEvent) -> None:
        self.update_completer_model()  # update completer model when entering
        self._old_index = self.currentIndex()
        super().focusInEvent(e)

    @QtCore.pyqtSlot()
    def update_completer_model(self):
        if self._completer is not None:
            self._completer.setModel(self.model().allRefs())

    @QtCore.pyqtSlot()
    def onFinishedEditing(self):
        m = self.model()
        cnt = self._completer.completionCount()
        text = self.currentText()
        idx = self.findData(text, Qt.DisplayRole)
        if idx >= 0:  # name is already present
            self.setCurrentIndex(idx)
        elif cnt == 1 or text in m._robot_state.robot_model.links.keys():
            # unique or perfect match with a robot link name
            text = self._completer.currentCompletion()
            m.add_ref(JointFrame(m._robot_state, text), text)
            self.setCurrentIndex(self.count() - 1)
        else:  # no or ambiguous match
            self.setCurrentIndex(self._old_index)

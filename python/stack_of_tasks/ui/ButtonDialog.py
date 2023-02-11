from PyQt5.QtCore import QAbstractListModel, QModelIndex, QSize, Qt, pyqtSignal
from PyQt5.QtWidgets import QDialog, QDialogButtonBox, QVBoxLayout

from stack_of_tasks.ui.data_input import DataInput


class ButtonDialog(QDialog):
    def __init__(self, parent: None) -> None:
        super().__init__(parent)

        self._vl = QVBoxLayout()
        self.setLayout(self._vl)

        self.buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Abort)

        self._vl.addWidget(self.buttonBox)

        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

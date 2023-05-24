from __future__ import annotations

from PyQt5.QtCore import QVariant, pyqtProperty, pyqtSignal
from PyQt5.QtWidgets import QComboBox

from stack_of_tasks.ui.model import RawDataRole
from stack_of_tasks.ui.model.object_model import ObjectModel


class ObjectDropdown(QComboBox):
    current_object_changed = pyqtSignal(object)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.object_model = ObjectModel()
        self.setModel(self.object_model)

        self.currentIndexChanged.connect(
            lambda index: self.current_object_changed.emit(self.currentData(RawDataRole))
        )

    def _get_co(self):
        x = self.currentData(RawDataRole)
        print(type(x))
        return x

    def _set_co(self, val):
        if val is not None:
            row = self.model().rowOf(val)
            self.setCurrentIndex(row)
            self.current_object_changed.emit(val)

    current_object = pyqtProperty(
        QVariant, _get_co, _set_co, notify=current_object_changed, user=True
    )

from __future__ import annotations

from typing import Callable, Generic, List, Optional, TypeVar

import PyQt5.QtWidgets as QtW
from PyQt5.QtCore import QAbstractListModel, QModelIndex, Qt, pyqtProperty, pyqtSignal
from PyQt5.QtWidgets import QComboBox, QWidget

from stack_of_tasks.ui.model import RawDataRole
from stack_of_tasks.ui.model.object_model import RawDataRole

ObjectType = TypeVar("ObjectType")


class ObjectModel(Generic[ObjectType], QAbstractListModel):
    def __init__(
        self,
        parent=None,
        data: Optional[List[ObjectType]] = None,
        disp_func: Optional[Callable] = None,
    ) -> None:
        super().__init__(parent)

        self._objs: List[ObjectType] = [] if data is None else data
        self._disp_func = disp_func

    def rowOf(self, object: ObjectType) -> int:
        return self._objs.index(object)

    def rowCount(self, parent=None) -> int:
        return len(self._objs)

    def get_display_string(self, index: QModelIndex) -> str:
        if self._disp_func is not None:
            return self._disp_func(self._objs[index.row()])

        return str(self._objs[index.row()])

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole) -> str | ObjectType | None:
        if role == Qt.DisplayRole:
            return self.get_display_string(index)
        elif role == RawDataRole:
            return self._objs[index.row()]


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
        return self.currentData(RawDataRole)

    def _set_co(self, val):
        self.setCurrentIndex(self.model().rowOf(val))
        self.current_object_changed.emit(val)

    current_object = pyqtProperty(
        str, _get_co, _set_co, notify=current_object_changed, user=True
    )

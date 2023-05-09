from __future__ import annotations

from typing import Callable, Generic, List, Optional, TypeVar

import PyQt5.QtWidgets as QtW
from PyQt5.QtCore import QAbstractListModel, QModelIndex, Qt
from PyQt5.QtWidgets import QWidget

from stack_of_tasks.ui.model import RawDataRole

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

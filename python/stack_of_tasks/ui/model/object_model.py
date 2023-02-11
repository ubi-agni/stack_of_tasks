from __future__ import annotations

from typing import Generic, List, Optional, TypeVar

from PyQt5.QtCore import QAbstractListModel, QModelIndex, Qt

from . import RawDataRole

ObjectType = TypeVar("ObjectType")


class ObjectModel(Generic[ObjectType], QAbstractListModel):
    def __init__(self, parent=None, data: Optional[List[ObjectType]] = None) -> None:
        super().__init__(parent)

        self._additional_data = []
        self._objs: List[ObjectType] = [] if data is None else data

    def rowCount(self, parent=None) -> int:
        return len(self._objs)

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole) -> str | ObjectType | None:
        if role == Qt.DisplayRole:
            return str(self._additional_data[index.row()]["name"])
        elif role == RawDataRole:
            return self._objs[index.row()]

    def add_object(self, obj: ObjectType, **additional_data):
        self.beginInsertRows(QModelIndex(), len(self._objs), len(self._objs))
        self._additional_data.append(additional_data)
        self._objs.append(obj)
        self.endInsertRows()

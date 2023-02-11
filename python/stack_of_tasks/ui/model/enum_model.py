from PyQt5.QtCore import Qt, QAbstractListModel, QModelIndex
from enum import Enum
from . import RawDataRole


class EnumModel(QAbstractListModel):
    def __init__(self, enum: Enum, parent=None) -> None:
        super().__init__(parent)
        self._types = list(enum)

    def rowCount(self, _) -> int:
        return len(self._types)

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole):
        if role == Qt.DisplayRole:
            return self._types[index.row()].name
        elif role == RawDataRole:
            return self._types[index.row()]

from __future__ import annotations

import typing

import numpy as np
from PyQt5.QtCore import QAbstractTableModel, QModelIndex, Qt


class NumpyTableModel(QAbstractTableModel):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self._matrix: np.ndarray = np.identity(4)

    def rowCount(self, parent: QModelIndex = None) -> int:
        return self._matrix.shape[0]

    def columnCount(self, parent: QModelIndex = None) -> int:
        return self._matrix.shape[1]

    def flags(self, index: QModelIndex) -> Qt.ItemFlags:
        return super().flags(index) | Qt.ItemIsEditable

    def data(self, index: QModelIndex, role: int = ...) -> typing.Any:
        if index.isValid():
            if role == Qt.DisplayRole:
                return str(self._matrix[index.row(), index.column()])

            elif role == Qt.EditRole:
                return self._matrix[index.row(), index.column()]

    def valuesChanged(self):
        self.modelAboutToBeReset.emit()
        self.dataChanged.emit(
            self.index(0, 0), self.index(self.rowCount(), self.columnCount())
        )
        self.modelReset.emit()

    def setData(self, index: QModelIndex, value: typing.Any, role: int = ...) -> bool:
        self._matrix[index.row(), index.column()] = value
        self.dataChanged.emit(index, index)
        return True

    def setMatrix(self, matrix):
        self.modelAboutToBeReset.emit()
        self._matrix = matrix
        self.modelReset.emit()

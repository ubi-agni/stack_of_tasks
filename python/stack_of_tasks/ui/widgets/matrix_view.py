from __future__ import annotations

from typing import Any, Optional

import numpy as np
from PyQt5.QtCore import (
    QAbstractTableModel,
    QMargins,
    QModelIndex,
    QSize,
    Qt,
    QVariant,
    pyqtProperty,
    pyqtSignal,
)
from PyQt5.QtGui import QClipboard, QKeyEvent
from PyQt5.QtWidgets import QApplication, QFrame, QHeaderView, QSizePolicy, QTableView

FloatDataRole = Qt.UserRole + 1

if not hasattr(QSize, "grownBy"):

    def grownBy(self: QSize, margins: QMargins):
        return QSize(
            self.width() + margins.left() + margins.right(),
            self.height() + margins.top() + margins.bottom(),
        )

    setattr(QSize, "grownBy", grownBy)


class NumpyTableModel(QAbstractTableModel):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self._matrix: Optional[np.ndarray] = None

    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return 0 if parent.isValid() or self._matrix is None else self._matrix.shape[0]

    def columnCount(self, parent: QModelIndex = QModelIndex()) -> int:
        if parent.isValid() or self._matrix is None:
            return 0
        return 1 if self._matrix.ndim == 1 else self._matrix.shape[1]

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole) -> Any:
        if not index.isValid() or self._matrix is None:
            return None
        if not (0 <= index.row() < self.rowCount() and 0 <= index.column() < self.columnCount()):
            return None
        if role in (Qt.DisplayRole, Qt.EditRole):
            if self._matrix.ndim == 1:
                value = self._matrix[index.row()]
            else:
                value = self._matrix[index.row(), index.column()]
            if role == Qt.DisplayRole:
                return f"{value:.2f}"
            return value.item() if isinstance(value, np.generic) else value
        return None

    def flags(self, index: QModelIndex) -> Qt.ItemFlags:
        flags = super().flags(index)
        return flags | Qt.ItemIsEditable if index.isValid() else flags

    def setData(self, index: QModelIndex, value: Any, role: int = Qt.EditRole) -> bool:
        if (
            role != Qt.EditRole
            or self._matrix is None
            or not index.isValid()
            or not (0 <= index.row() < self.rowCount() and 0 <= index.column() < self.columnCount())
        ):
            return False
        key = index.row() if self._matrix.ndim == 1 else (index.row(), index.column())
        if self._matrix[key] != value:
            self._matrix[key] = value
            self.dataChanged.emit(index, index)
        return True

    def setMatrix(self, matrix: np.ndarray):
        self.beginResetModel()
        self._matrix = matrix
        self.endResetModel()

    def setMatrixData(self, data, startIndex: QModelIndex, endIndex: Optional[QModelIndex] = None):
        if self._matrix is not None:
            if endIndex is None:
                index = (
                    slice(startIndex.row(), startIndex.row() + data.shape[0]),
                    slice(startIndex.column(), startIndex.column() + data.shape[1]),
                )
            else:
                index = (
                    slice(startIndex.row(), endIndex.row() + 1),
                    slice(startIndex.column(), endIndex.column() + 1),
                )

            matrix = self._matrix[:, np.newaxis] if self._matrix.ndim == 1 else self._matrix
            matrix[index] = data
            row_start, row_stop, _ = index[0].indices(self.rowCount())
            column_start, column_stop, _ = index[1].indices(self.columnCount())
            if row_start < row_stop and column_start < column_stop:
                self.dataChanged.emit(
                    self.index(row_start, column_start), self.index(row_stop - 1, column_stop - 1)
                )

    def matrix(self):
        return self._matrix


class MatrixWidget(QTableView):
    matrixChanged = pyqtSignal(object)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        self.setSizeAdjustPolicy(self.AdjustToContents)

        self.verticalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.verticalHeader().setVisible(False)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.horizontalHeader().setVisible(False)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.setFrameStyle(QFrame.NoFrame)
        self.minimum_size = None

        self.matrix_model = NumpyTableModel()
        self.setModel(self.matrix_model)
        self.matrix_model.dataChanged.connect(lambda: self.matrixChanged.emit(self.get_matrix()))

    def minimumSizeHint(self) -> QSize:
        if self.minimum_size is None:
            # compute minimum size
            self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
            self.minimum_size = self.sizeHint()
            self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        return self.minimum_size

    def model(self) -> NumpyTableModel:
        return super().model()

    def keyPressEvent(self, e: QKeyEvent) -> None:
        if e.modifiers() & Qt.ControlModifier:
            if e.key() == Qt.Key_V:
                text = QApplication.clipboard().text()
                data = [r.split("\t") for r in text.split("\n")]
                data = np.array(data, dtype=float)

                sel = self.selectedIndexes()
                if len(sel) == 1:
                    self.model().setMatrixData(data, sel[0])
                else:
                    self.model().setMatrixData(data, sel[0], sel[-1])
                e.accept()

            elif e.key() == Qt.Key_C:
                ind = self.selectedIndexes()
                line_strings = []

                for line in self.get_matrix()[ind[0].row() : ind[-1].row() + 1]:
                    line_strings.append(
                        "\t".join(line[ind[0].column() : ind[-1].column() + 1].astype(str))
                    )
                array_string = "\n".join(line_strings)
                QApplication.clipboard().setText(array_string, QClipboard.Clipboard)
                e.accept()

        if not e.isAccepted():
            return super().keyPressEvent(e)

    def get_matrix(self):
        return self.matrix_model.matrix()

    def set_matrix(self, val):
        self.matrix_model.setMatrix(val)
        self.minimum_size = None

    matrix = pyqtProperty(QVariant, get_matrix, set_matrix, notify=matrixChanged, user=True)

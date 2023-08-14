from __future__ import annotations

from typing import Any, Optional

import numpy as np
from PyQt5.QtCore import (
    QMargins,
    QModelIndex,
    QObject,
    QSize,
    Qt,
    QVariant,
    pyqtProperty,
    pyqtSignal,
)
from PyQt5.QtGui import QClipboard, QKeyEvent, QStandardItemModel
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFrame,
    QHeaderView,
    QSizePolicy,
    QStyle,
    QStyledItemDelegate,
    QStyleOptionViewItem,
    QTableView,
    QWidget,
)

from stack_of_tasks.ui.property_tree.base import BaseItem, RawDataRole

FloatDataRole = Qt.UserRole + 1

if not hasattr(QSize, "grownBy"):

    def grownBy(self: QSize, margins: QMargins):
        return QSize(
            self.width() + margins.left() + margins.right(),
            self.height() + margins.top() + margins.bottom(),
        )

    setattr(QSize, "grownBy", grownBy)


class MatrixItem(BaseItem):
    def __init__(self, i, j):
        super().__init__()
        self._i = i
        self._j = j

    def model(self) -> NumpyTableModel:
        return super().model()

    def raw_data(self):
        return self.model().matrix()[self._i, self._j]

    def data(self, role: int = ...) -> Any:
        if role == Qt.DisplayRole:
            return f"{self.raw_data():.2f}"

        return super().data(role)

    def setData(self, value: Any, role: int = ...) -> bool:
        if role == RawDataRole:
            self.model().matrix()[self._i, self._j] = value
        else:
            return super().setData(value, role)


class NumpyTableModel(QStandardItemModel):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self._matrix: np.ndarray = None
        self._is_1D: bool

    def valuesChanged(self):
        self.dataChanged.emit(
            self.index(0, 0), self.index(self.rowCount(), self.columnCount())
        )

    def _setup_items(self):
        shape = self._matrix.shape
        self._is_1D = len(shape) == 1

        if self._is_1D:
            for i in range(shape[0]):
                self.setItem(i, 0, MatrixItem(i, 0))
        else:
            for i in range(shape[0]):
                for j in range(shape[1]):
                    self.setItem(i, j, MatrixItem(i, j))

    def setMatrix(self, matrix: np.ndarray):
        if self._matrix is None or self._matrix.shape != matrix.shape:
            self._matrix = matrix
            self._setup_items()

        else:
            self._matrix = matrix
            self.valuesChanged()

    def setMatrixData(self, data, startIndex: QModelIndex, endIndex: QModelIndex = None):
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

            print(index, data.shape)

            self._matrix[index] = data
            self._setup_items()

    def matrix(self):
        return self._matrix


class MatrixItemDelegate(QStyledItemDelegate):
    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)

        self._editor_widget = QDoubleSpinBox()
        self._editor_widget.setDecimals(10)

        self._editor_widget.setFocusPolicy(Qt.StrongFocus)
        self._editor_widget.setAutoFillBackground(True)

    def createEditor(
        self, parent: QWidget, option: QStyleOptionViewItem, index: QModelIndex
    ) -> QWidget:
        self._editor_widget.setParent(parent)
        return self._editor_widget

    def setModelData(
        self, editor: QDoubleSpinBox, model: QStandardItemModel, index: QModelIndex
    ) -> None:
        model.setData(index, editor.value(), RawDataRole)

    def setEditorData(self, editor: QDoubleSpinBox, index: QModelIndex) -> None:
        f = index.data(RawDataRole)
        editor.setValue(f)

    def updateEditorGeometry(
        self, editor: QDoubleSpinBox, option: QStyleOptionViewItem, index: QModelIndex
    ) -> None:
        editor.move(option.rect.topLeft())

    def destroyEditor(self, editor: QDoubleSpinBox, index: QModelIndex) -> None:
        editor.hide()

    def sizeHint(self, option: QStyleOptionViewItem, index: QModelIndex) -> QSize:
        if option.state & QStyle.State_Editing:
            return self._editor_widget.minimumSize()
        else:
            return super().sizeHint(option, index)


class MatrixWidget(QTableView):
    matrix_changed = pyqtSignal(object)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        self.setSizeAdjustPolicy(self.AdjustToContents)

        self.verticalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.verticalHeader().setMinimumSectionSize(0)

        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.horizontalHeader().setMinimumSectionSize(0)
        self.horizontalHeader().setVisible(False)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.verticalHeader().setVisible(False)

        self.setItemDelegate(MatrixItemDelegate())
        self.viewport().setMinimumSize(0, 0)

        self.setFrameStyle(QFrame.NoFrame)

        self.matrix_model = NumpyTableModel()
        self.setModel(self.matrix_model)

        self.matrix_changed.connect(self._mc)

    def model(self) -> NumpyTableModel:
        return super().model()

    def _mc(self, data):
        print("matrix changed: ", data)

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

    def dataChanged(self, topLeft: QModelIndex, bottomRight: QModelIndex, roles=None) -> None:
        pass
        # self.matrix_changed.emit(self.get_matrix())
        # super().dataChanged(topLeft, bottomRight, roles)

    def get_matrix(self):
        return self.matrix_model.matrix()

    def set_matrix(self, val):
        self.matrix_model.setMatrix(val)

    matrix = pyqtProperty(QVariant, get_matrix, set_matrix, notify=matrix_changed, user=True)
    matrix = pyqtProperty(QVariant, get_matrix, set_matrix, notify=matrix_changed, user=True)

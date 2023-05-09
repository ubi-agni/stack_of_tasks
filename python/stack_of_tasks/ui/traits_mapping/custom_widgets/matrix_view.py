from __future__ import annotations

import typing

import numpy as np
from PyQt5 import QtCore
from PyQt5.QtCore import (
    QAbstractTableModel,
    QMargins,
    QModelIndex,
    QObject,
    QRectF,
    QSize,
    Qt,
)
from PyQt5.QtGui import QPainter, QTextOption
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QHeaderView,
    QStyledItemDelegate,
    QStyleOptionViewItem,
    QTableView,
    QWidget,
)

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

            elif role == FloatDataRole:
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


class MatrixItemDelegate(QStyledItemDelegate):
    def __init__(self, parent: typing.Optional[QObject] = None) -> None:
        super().__init__(parent)

        self._editor_widget = QDoubleSpinBox()
        self._editor_index = {}

        self._editor_widget.setFocusPolicy(Qt.StrongFocus)
        self._editor_widget.setDecimals(10)
        self._editor_widget.setAutoFillBackground(True)

    def _format_index(self, index: QModelIndex):
        s = ""
        if index.isValid() and (f := index.data(FloatDataRole)) is not None:
            s = f"{f:.2f}"
        return s

    def createEditor(
        self, parent: QWidget, option: QStyleOptionViewItem, index: QModelIndex
    ) -> QWidget:
        self._editor_widget.setParent(parent)
        self._editor_index[index] = option.widget
        option.widget.resizeColumnToContents(index.column())
        return self._editor_widget

    def setEditorData(self, editor: QWidget, index: QModelIndex) -> None:
        f = index.data(FloatDataRole)
        self._editor_widget.setValue(f)

    def updateEditorGeometry(
        self, editor: QWidget, option: QStyleOptionViewItem, index: QModelIndex
    ) -> None:
        editor.move(option.rect.topLeft())

    def destroyEditor(self, editor: QWidget, index: QModelIndex) -> None:
        i = self._editor_index.pop(index, None)
        if i is not None:
            i.resizeColumnToContents(index.column())
        self._editor_widget.hide()

    def sizeHint(self, option: QStyleOptionViewItem, index: QModelIndex) -> QSize:
        if index in self._editor_index:
            return self._editor_widget.sizeHint()
        else:
            size = option.fontMetrics.size(0, self._format_index(index))
            return size.grownBy(QMargins(3, 3, 3, 3))

    def paint(
        self, painter: QPainter, option: QStyleOptionViewItem, index: QModelIndex
    ) -> None:
        painter.save()
        r = QRectF(option.rect)
        painter.drawText(
            r,
            self._format_index(index),
            QTextOption(Qt.AlignCenter),
        )
        painter.restore()


class MatrixView(QTableView):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

    def setModel(self, model) -> None:
        super().setModel(model)
        self.verticalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.verticalHeader().setMinimumSectionSize(0)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.horizontalHeader().setMinimumSectionSize(0)
        self.setItemDelegate(MatrixItemDelegate())

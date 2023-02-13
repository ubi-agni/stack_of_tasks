from __future__ import annotations

import typing

from PyQt5.QtCore import QMargins, QModelIndex, QObject, QRectF, QSize, Qt
from PyQt5.QtGui import QPainter, QTextOption
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QHeaderView,
    QStyledItemDelegate,
    QStyleOptionViewItem,
    QTableView,
    QWidget,
)

if not hasattr(QSize, "grownBy"):

    def grownBy(self: QSize, margins: QMargins):
        return QSize(
            self.width() + margins.left() + margins.right(),
            self.height() + margins.top() + margins.bottom(),
        )

    setattr(QSize, "grownBy", grownBy)


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
        if index.isValid() and (f := index.data(Qt.EditRole)) is not None:
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
        f = index.data(Qt.EditRole)
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
    def __init__(self, parent=None, model=None) -> None:
        super().__init__(parent)
        if model is not None:
            self.setModel(model)

        self.setItemDelegate(MatrixItemDelegate())
        self.verticalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.verticalHeader().setMinimumSectionSize(0)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.horizontalHeader().setMinimumSectionSize(0)

        self

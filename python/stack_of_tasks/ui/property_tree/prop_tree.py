#!/usr/bin/env python3
from __future__ import annotations

from PyQt5.QtCore import QModelIndex, QRect, Qt
from PyQt5.QtGui import QDrag, QPainter, QPixmap, QStandardItemModel
from PyQt5.QtWidgets import QHeaderView, QStyleOptionViewItem, QTreeView, QWidget

from stack_of_tasks.logger import sot_logger
from stack_of_tasks.tasks import Task

from .prop_item_delegate import PropItemDelegate

logger = sot_logger.getChild("TaskView")


class SOT_View(QTreeView):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self.setItemDelegate(PropItemDelegate())

        self.header().setStretchLastSection(False)
        self.header().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.header().setVisible(False)

        self.setSelectionBehavior(self.SelectRows)
        self.setSelectionMode(self.ExtendedSelection)

        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.setDragDropMode(self.InternalMove)

    def model(self) -> QStandardItemModel:
        return super().model()

    def setModel(self, model) -> None:
        super().setModel(model)

        if model is not None:
            model.rowsInserted.connect(self.expand_levels)
            self.expand_levels(QModelIndex(), 0, model.rowCount() - 1)

    def expand_levels(self, parent: QModelIndex, first: int, last: int):
        if parent.isValid():
            return  # only expand top-level items
        for row in range(first, last + 1):
            self.expand(self.model().index(row, 0))

    def add_tasks(self, tasks: list[Task]):
        selected_indices = self.selectedIndexes()
        parent = self.rootIndex() if len(selected_indices) == 0 else selected_indices[0]

        self.model().insert_task(tasks[0], parent)

    def remove_selected(self):
        self.model().remove_task(self.selectedIndexes())

    def startDrag(self, supportedActions: Qt.DropActions | Qt.DropAction) -> None:

        index = self.selectedIndexes()[0]
        mimeData = self.model().mimeData(self.selectedIndexes())

        drag = QDrag(self)

        delegate = self.itemDelegate()
        style = QStyleOptionViewItem()

        delegate.initStyleOption(style, index)

        size = delegate.sizeHint(style, index)
        rect = QRect()
        rect.setSize(size)
        style.rect = rect

        pm = QPixmap(rect.size())
        pm.fill(Qt.transparent)

        painter = QPainter(pm)
        delegate.paint(painter, style, index)
        painter.end()

        drag.setPixmap(pm)
        drag.setMimeData(mimeData)
        drag.exec()

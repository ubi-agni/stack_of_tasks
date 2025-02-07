#!/usr/bin/env python3
from __future__ import annotations

import typing

import traits.observation.events as te
from PyQt5.QtCore import QMimeData, QModelIndex, QPersistentModelIndex, Qt, pyqtSignal
from PyQt5.QtGui import QStandardItem, QStandardItemModel
from PyQt5.QtWidgets import QHeaderView, QTreeView, QWidget

from stack_of_tasks.logger import sot_logger
from stack_of_tasks.tasks import Task, TaskHierarchy
from stack_of_tasks.ui.property_tree.base import PlaceholderItem
from stack_of_tasks.ui.property_tree.prop_item import AttrValueItem, LevelItem, TaskItem

logger = sot_logger.getChild("TaskView")


class InternalMoveMimeData(QMimeData):
    def __init__(self, parent: QModelIndex, row: int) -> None:
        super().__init__()
        self.parent_index = QPersistentModelIndex(parent)
        self.row = row


class SOT_Model(QStandardItemModel):

    level_inserted = pyqtSignal(list, int)
    task_inserted = pyqtSignal(Task, int)

    level_removed = pyqtSignal(int)
    task_removed = pyqtSignal(int, int)

    def insert_task(self, task: Task, parent: QModelIndex = None):
        new_level = False
        if parent.isValid():
            pitem = self.itemFromIndex(parent)
            if isinstance(pitem, LevelItem):
                level = pitem
            else:
                level = pitem.parent()
        else:
            level = LevelItem()
            self.appendRow([level, PlaceholderItem()])
            new_level = True

        level.appendRow([TaskItem(task), AttrValueItem(task, "residual")])
        if new_level:
            self.level_inserted.emit([task], level.row())
        else:
            self.task_inserted.emit(task, level.row())

    def init_model(self, stack_of_tasks: TaskHierarchy):
        for l in stack_of_tasks.levels:
            level = LevelItem()
            for t in l:
                level.appendRow([TaskItem(t), AttrValueItem(t, "residual")])
            self.appendRow([level, PlaceholderItem()])

    def remove_task(self, indexes: list[QModelIndex]):

        for index in indexes:
            p_index = index.parent()

            if p_index.isValid():
                # selected is task
                self.removeRow(index.row(), p_index)
                self.task_removed.emit(p_index.row(), index.row())

            # Remove level if empty
            if p_index.isValid() and self.rowCount(p_index) == 0:
                self.removeRow(p_index.row())
                self.level_removed.emit(p_index.row())

    def mimeData(self, indexes: typing.Iterable[QModelIndex]) -> QMimeData:
        parent: QModelIndex = indexes[0].parent()

        return InternalMoveMimeData(parent, indexes[0].row())

    def canDropMimeData(
        self,
        data: QMimeData,
        action: Qt.DropAction,
        row: int,
        column: int,
        parent: QModelIndex,
    ) -> bool:
        if isinstance(data, InternalMoveMimeData):
            if parent.parent().isValid() or (parent.isValid() and row > -1):
                # drop position is task or drop in level at position
                return False

            # print(parent.isValid(), self.itemFromIndex(parent), row)
        return True
        # return super().canDropMimeData(data, action, row, column, parent)

    def dropMimeData(
        self,
        data: QMimeData,
        action: Qt.DropAction,
        row: int,
        column: int,
        dest_parent: QModelIndex,
    ) -> bool:
        if isinstance(data, InternalMoveMimeData):
            source_parent = QModelIndex(data.parent_index)
            dest_is_level = False

            if row == -1:
                row = self.rowCount(dest_parent)

            if not dest_parent.isValid():
                dest_parent = QModelIndex()
                dest_parent_item = self.invisibleRootItem()
            else:
                dest_parent_item = self.itemFromIndex(dest_parent)
                dest_is_level = True

            if source_parent.isValid():
                # move task ...
                source_items = self.itemFromIndex(source_parent).takeRow(data.row)
                self.task_removed.emit(source_parent.row(), data.row)
                if dest_is_level:
                    # ... from one level to another
                    dest_parent_item.insertRow(row, source_items)
                    print(dest_parent_item.rowCount())
                    self.task_inserted.emit(source_items[0]._obj, dest_parent_item.row())
                else:
                    # ... into a new level
                    level = LevelItem()
                    level.appendRow(source_items)
                    self.insertRow(row, [level, PlaceholderItem()])

                    self.level_inserted.emit([source_items[0]._obj], row)

                if self.rowCount(source_parent) == 0:
                    # remove source level if empty
                    self.removeRow(source_parent.row())
                    self.level_removed.emit(source_parent.row())

            else:
                # move level...
                if dest_is_level:
                    # ... insert its task into destination level
                    source_level = self.item(data.row)
                    count = source_level.rowCount()

                    for i in range(count):
                        si = source_level.takeRow(0)
                        self.task_removed.emit(source_level.row(), 0)

                        dest_parent_item.insertRow(row, si)

                        self.task_inserted.emit(si[0]._obj, dest_parent.row())

                    self.removeRow(data.row)
                    self.level_removed.emit(data.row)
                else:
                    # ... to its new position
                    source_items = self.takeRow(data.row)
                    self.insertRow(row, source_items)

            return True
        else:
            return super().dropMimeData(data, action, row, column, dest_parent)

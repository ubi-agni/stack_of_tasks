#!/usr/bin/env python3
from __future__ import annotations

import typing

import PyQt5.QtWidgets as QtW
import traits.observation.events as te
from PyQt5.QtCore import QMimeData, QModelIndex, QPersistentModelIndex, Qt
from PyQt5.QtGui import QMouseEvent, QStandardItem, QStandardItemModel
from PyQt5.QtWidgets import QHeaderView, QWidget

from stack_of_tasks.logger import sot_logger
from stack_of_tasks.tasks.Task import Task
from stack_of_tasks.tasks.TaskHierarchy import (
    ChangeType,
    HierarchieChanged,
    LevelChanged,
    TaskHierarchy,
)
from stack_of_tasks.ui import RawDataRole
from stack_of_tasks.ui.property_tree.base import PlaceholderItem

from .prop_item import AttrNameItem, AttrValueItem, LevelItem, RawDatatItem, TraitItem
from .prop_item_delegate import PropItemDelegate

logger = sot_logger.getChild("TaskView")


class InternalMoveMimeData(QMimeData):
    def __init__(self, source_item) -> None:
        super().__init__()
        self.source_item = source_item


class SOT_Model(QStandardItemModel):
    def __init__(self, stack_of_tasks: TaskHierarchy = None):
        super().__init__()

        self.stack_of_tasks = None

        if stack_of_tasks is not None:
            self.set_stack(stack_of_tasks)

    def set_stack(self, stack_of_tasks: TaskHierarchy):
        self.clear()
        self.stack_of_tasks = stack_of_tasks
        self.stack_of_tasks.observe(self._levels_changed, "layout_changed", dispatch="ui")

        self.layoutAboutToBeChanged.emit()

        for level in stack_of_tasks.levels:
            l = self._create_level(level)
            self.appendRow([l, PlaceholderItem()])

        self.layoutChanged.emit()

    def _create_level(self, task_list: list[Task]):
        l = LevelItem()
        for task in task_list:
            tn = TraitItem(task)

            l.appendRow([tn, PlaceholderItem()])

        return l

    def _levels_changed(self, evt: te.TraitChangeEvent):
        evt = evt.new
        if isinstance(evt, HierarchieChanged):
            if evt.type is ChangeType.removed:
                self.removeRow(evt.index)

            if evt.type is ChangeType.added:
                level = self.stack_of_tasks.levels[evt.index]
                self.appendRow([self._create_level(level), PlaceholderItem()])

        elif isinstance(evt, LevelChanged):
            level_index = self.index(evt.level, 0)
            levelItem = self.itemFromIndex(level_index)

            if evt.type is ChangeType.added:
                for task in evt.tasks:
                    task_item = TraitItem(task)
                    levelItem.appendRow([task_item, PlaceholderItem()])

            if evt.type is ChangeType.removed:
                for task in evt.tasks:
                    for cid in range(levelItem.rowCount()):
                        if self.data(self.index(cid, 0, level_index), RawDataRole) is task:
                            levelItem.removeRow(cid)
                            break

    def remove_item_at_index(self, index_to_remove: QModelIndex):
        item = self.itemFromIndex(index_to_remove)

        if isinstance(item, LevelItem):
            level = item.row()
            self.stack_of_tasks.remove_level(level)

        elif isinstance(item, RawDatatItem):
            task = item.data(RawDataRole)
            self.stack_of_tasks.remove_task(task)

    def add_task(self, task: Task, parent: QModelIndex = None):
        if parent is None:
            with self.stack_of_tasks.new_level() as l:
                l.append(task)
        else:
            parent_item = self.itemFromIndex(parent)

            if parent_item is not None:
                if isinstance(parent_item, LevelItem):
                    self.stack_of_tasks.levels[parent_item.row()].append(task)
                elif isinstance(parent_item, TraitItem):
                    self.stack_of_tasks.levels[parent_item.parent().row()].append(task)

    def mimeData(self, indexes: typing.Iterable[QModelIndex]) -> QMimeData:
        item = self.itemFromIndex(indexes[0])

        return InternalMoveMimeData(item)

    def canDropMimeData(
        self,
        data: QMimeData,
        action: Qt.DropAction,
        row: int,
        column: int,
        parent: QModelIndex,
    ) -> bool:
        if isinstance(data, InternalMoveMimeData):
            parent_item = self.itemFromIndex(parent)
            return column <= 0 and (isinstance(parent_item, LevelItem) or parent_item is None)

        return super().canDropMimeData(data, action, row, column, parent)

    def dropMimeData(
        self,
        data: QMimeData,
        action: Qt.DropAction,
        row: int,
        column: int,
        parent: QModelIndex,
    ) -> bool:
        if isinstance(data, InternalMoveMimeData):
            source_item: QStandardItem = data.source_item
            dest_item = self.itemFromIndex(parent)

            if isinstance(source_item, LevelItem) and dest_item is None:
                self.stack_of_tasks.move_level(source_item.row(), row)

            elif isinstance(source_item, RawDatatItem):
                task = source_item.data(RawDataRole)

                if isinstance(dest_item, LevelItem):
                    level = dest_item.row()
                    self.stack_of_tasks.move_task_to_level(level, task)

                elif dest_item is None:
                    self.stack_of_tasks.remove_task(task)
                    self.stack_of_tasks.insert_level(row, [task])

            return True
        else:
            return super().dropMimeData(data, action, row, column, parent)


class SOT_View(QtW.QTreeView):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self.setModel(SOT_Model())
        self.setItemDelegate(PropItemDelegate())

        self.header().setStretchLastSection(False)
        self.header().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.header().setVisible(False)

        self.setSelectionBehavior(self.SelectItems)
        self.setSelectionMode(self.SingleSelection)

        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.setDragDropMode(self.InternalMove)

    def model(self) -> SOT_Model:
        return super().model()

    def mousePressEvent(self, e: QMouseEvent) -> None:
        self.clearSelection()
        return super().mousePressEvent(e)

    def add_task(self, task: Task):
        selected_indices = self.selectedIndexes()
        self.model().add_task(
            task, None if len(selected_indices) == 0 else selected_indices[0]
        )

    def remove_selected(self):
        selected_indices = self.selectedIndexes()
        self.model().remove_item_at_index(selected_indices[0])

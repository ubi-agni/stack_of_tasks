#!/usr/bin/env python3
from __future__ import annotations

import typing

import traits.observation.events as te
from PyQt5.QtCore import QMimeData, QModelIndex, Qt
from PyQt5.QtGui import QStandardItem, QStandardItemModel
from PyQt5.QtWidgets import QHeaderView, QTreeView, QWidget

from stack_of_tasks.logger import sot_logger
from stack_of_tasks.tasks import Task, TaskHierarchy
from stack_of_tasks.ui.property_tree.base import PlaceholderItem

from .prop_item import AttrValueItem, LevelItem, TaskItem
from .prop_item_delegate import PropItemDelegate

logger = sot_logger.getChild("TaskView")


class InternalMoveMimeData(QMimeData):
    def __init__(self, items: typing.Iterable[QStandardItem]) -> None:
        super().__init__()
        self.items = items


class SOT_Model(QStandardItemModel):
    def __init__(self, task_hierarchy: TaskHierarchy = None):
        super().__init__()
        self.set_task_hierarchy(task_hierarchy)

    def set_task_hierarchy(self, task_hierarchy: TaskHierarchy):
        self.task_hierarchy = task_hierarchy
        self.reset_model()
        self.task_hierarchy.observe(self.reset_model, "levels")
        self.task_hierarchy.observe(self._levels_changed, "levels:items")
        self.task_hierarchy.observe(self._tasks_changed, "levels:items:items")

    def reset_model(self, *args):
        self.clear()
        self.insert_level_rows(levels=self.task_hierarchy.levels)

    def insert_level_rows(self, levels: list[list[Task]], *, row: int = -1):
        """Create model rows for the given levels"""
        if row < 0:
            row = self.rowCount()

        for i, tasks in enumerate(levels):
            level = LevelItem()
            self.insert_task_rows(level, tasks)
            self.insertRow(row + i, [level, PlaceholderItem()])

    def insert_task_rows(self, level: LevelItem, tasks: list[Task], *, row: int = -1):
        """Create model rows for the given tasks below the given level item"""
        if row < 0:
            row = level.rowCount()

        for i, task in enumerate(tasks):
            level.insertRow(row + i, [TaskItem(task), AttrValueItem(task, "residual")])

    def level_index(self, level: list[Task]) -> QModelIndex:
        """Get the model index of the given level within the hierarchy"""
        row = self.task_hierarchy.levels.index(level)
        return self.index(row, 0)

    def _levels_changed(self, event: te.ListChangeEvent):
        """Handle changes in the task hierarchy's levels list"""
        self.removeRows(event.index, len(event.removed))
        self.insert_level_rows(event.added, row=event.index)

    def _tasks_changed(self, event: te.ListChangeEvent):
        """Handle changes in a level's tasks list"""
        level = self.itemFromIndex(self.level_index(event.object))
        level.removeRows(event.index, len(event.removed))
        self.insert_task_rows(level, event.added, row=event.index)
        if len(event.object) == 0:
            self.task_hierarchy.levels.remove(event.object)

    def remove(self, indexes: list[QModelIndex]):
        """Remove the items at the given indexes from the model"""
        for item in list(map(self.itemFromIndex, indexes)):
            if isinstance(item, LevelItem):
                del self.task_hierarchy.levels[item.row()]

            elif isinstance(item, TaskItem):
                del self.task_hierarchy.levels[item.parent().row()][item.row()]

    @staticmethod
    def insert_into_level(level: list[Task], tasks: list[Task], row: int = -1):
        if row < 0 or row >= len(level):  # just append all tasks
            level.extend(tasks)
            return
        for i, t in enumerate(tasks):  # insert tasks at given row
            level.insert(row + i, t)

    def add_tasks(self, tasks: list[Task], parent: QModelIndex = None):
        if not parent.isValid():  # append at root
            self.task_hierarchy.levels.append(tasks)
        else:
            parent = self.itemFromIndex(parent)
            if isinstance(parent, LevelItem):  # append at level
                self.task_hierarchy.levels[parent.row()].extend(tasks)
            elif isinstance(parent, TaskItem):  # insert after task
                self.insert_into_level(
                    self.task_hierarchy.levels[parent.parent().row()], tasks, parent.row() + 1
                )

    def mimeData(self, indexes: typing.Iterable[QModelIndex]) -> QMimeData:
        return InternalMoveMimeData([self.itemFromIndex(idx) for idx in indexes])

    def canDropMimeData(
        self,
        data: QMimeData,
        action: Qt.DropAction,
        row: int,
        column: int,
        parent: QModelIndex,
    ) -> bool:
        if isinstance(data, InternalMoveMimeData):
            parent = self.itemFromIndex(parent)
            movable = any(isinstance(item, (LevelItem, TaskItem)) for item in data.items)
            print(movable, "->", parent, row, column)
            return (
                column <= 0 and (isinstance(parent, LevelItem) or parent is None) and movable
            )
        return super().canDropMimeData(data, action, row, column, parent)

    def dropMimeData(
        self,
        data: QMimeData,
        action: Qt.DropAction,
        row: int,
        column: int,
        parent: QModelIndex,
    ) -> bool:
        have_inserted = False
        if isinstance(data, InternalMoveMimeData):
            levels = self.task_hierarchy.levels
            dest = self.itemFromIndex(parent)

            def insert(tasks: list[Task], append_on_level=False):
                if dest is None:  # drop on root
                    # BUG: list insertion disables trait notifications
                    # idx = len(levels) if row < 0 else row
                    # levels.insert(idx, tasks)
                    idx = len(levels)
                    levels.append(tasks)
                    return self.itemFromIndex(self.index(idx, 0))
                elif isinstance(dest, LevelItem):
                    if row < 0:  # drop on level
                        if append_on_level:
                            self.insert_into_level(levels[dest.row()], tasks)
                        else:  # insert after
                            return None  # BUG: list insertion disables trait notifications
                            idx = dest.row() + 1
                            levels.insert(idx, tasks)
                            return self.itemFromIndex(self.index(idx, 0))
                    else:  # drop between tasks: insert at given row
                        self.insert_into_level(levels[dest.row()], tasks, row)
                elif isinstance(dest, TaskItem) and row < 0:  # drop on task: insert after
                    self.insert_into_level(levels[dest.parent().row()], tasks, dest.row() + 1)
                else:
                    return None  # indicate failure
                return dest  # return level item we inserted too

            for item in filter(lambda x: isinstance(x, LevelItem), data.items):
                old_text = item.data(Qt.EditRole)
                tasks = levels.pop(item.row())
                item = insert(tasks)
                have_inserted |= item is not None
                if isinstance(item, LevelItem):
                    item.setData(old_text, Qt.EditRole)

            for item in filter(lambda x: isinstance(x, TaskItem), data.items):
                prev_level = levels[item.parent().row()]
                task = prev_level.pop(item.row())
                new_dest = insert([task], append_on_level=True)
                have_inserted |= new_dest is not None

                if new_dest is not None and new_dest is not dest:
                    dest = new_dest
                    row = -1  # append

        return super().dropMimeData(data, action, row, column, parent) or have_inserted


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
        parent = QModelIndex() if len(selected_indices) == 0 else selected_indices[0]
        self.model().add_tasks(tasks, parent)

    def remove_selected(self):
        self.model().remove(self.selectedIndexes())

from __future__ import annotations

from typing import Any, Iterable, List, Optional, Sized, Union

import traits.trait_list_object as tal
from PyQt5.QtCore import QAbstractItemModel, QMimeData, QModelIndex, Qt

from stack_of_tasks.tasks.Task import Task
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy

from . import RawDataRole


class MoveMimeData(QMimeData):
    mimeType = "application/ModelIndexMime"

    def __init__(self, selected_indices) -> None:
        super().__init__()
        self.selected_indices = selected_indices

    def formats(self) -> List[str]:
        return [MoveMimeData.mimeType]


class TaskHierarchyModel(QAbstractItemModel):
    def __init__(self, task_hierarchy: Optional[TaskHierarchy] = None) -> None:
        super().__init__()

        self.task_hierarchy: TaskHierarchy
        self.set_hierarchy(task_hierarchy)

    def set_hierarchy(self, task_hierarchy: TaskHierarchy):
        self.beginResetModel()
        self.task_hierarchy = task_hierarchy
        self.endResetModel()

    # structure
    def rowCount(self, parent: QModelIndex = ...) -> int:
        if self.task_hierarchy is None:
            return 0

        parent_item = self.task_hierarchy

        if parent.isValid():
            parent_item = parent.internalPointer()

        if isinstance(parent_item, Task):
            return 0

        return len(parent_item)

    def columnCount(self, parent: QModelIndex = None) -> int:
        return 1

    def index(self, row: int, column: int, parent: QModelIndex = None) -> QModelIndex:
        parent_item: Sized = self.task_hierarchy

        if parent.isValid():
            parent_item = parent.internalPointer()

        if (child := self._get_child(parent_item, row)) is not None:
            return self.createIndex(row, column, child)

        return QModelIndex()

    def _get_child(self, item, row: int):
        if row < len(item):
            if isinstance(item, TaskHierarchy):
                return item.levels[row]
            elif isinstance(item, list):
                return item[row]

        return None

    def parent(self, index: QModelIndex):
        if index.isValid():
            item = index.internalPointer()

            if item in self.task_hierarchy:
                return QModelIndex()

            for i, l in enumerate(self.task_hierarchy.levels):
                if item in l:
                    return self.createIndex(i, 0, l)

        return QModelIndex()

    # data
    def flags(self, index: QModelIndex) -> Qt.ItemFlags:
        # flags = super().flags(index) | Qt.ItemIsSelectable | Qt.ItemIsDragEnabled
        # if not isinstance(index.internalPointer(), TaskItem):
        #    flags |= Qt.ItemIsDropEnabled
        return super().flags(index)

    def data(self, index: QModelIndex, role: int) -> Any:
        if index.isValid():
            item = index.internalPointer()

            if role == Qt.DisplayRole:
                if isinstance(item, list):
                    return f"Level "

                if isinstance(item, Task):
                    return f"{item.name} - "

            elif role == RawDataRole:
                if isinstance(item, Task):
                    return item

    def supportedDropActions(self) -> Qt.DropActions:
        return Qt.MoveAction

    # drag-drop
    def mimeData(self, indexes: Iterable[QModelIndex]) -> MoveMimeData:
        return MoveMimeData([i.internalPointer() for i in indexes])

    def canDropMimeData(
        self,
        data: MoveMimeData,
        action: Qt.DropAction,
        row: int,
        column: int,
        parent: QModelIndex,
    ) -> bool:
        # TODO better rejection
        if isinstance(data, MoveMimeData):
            return True
        return False

    def dropMimeData(
        self,
        data: MoveMimeData,
        action: Qt.DropAction,
        row: int,
        column: int,
        parent: QModelIndex,
    ) -> bool:
        target = self.task_hierarchy

        if parent.isValid():
            target = parent.internalPointer()

        if isinstance(target, TaskLevel) and all(
            target == source.parent for source in data.selected_indices
        ):
            return False

        self.layoutAboutToBeChanged.emit()

        for source in data.selected_indices:
            if isinstance(target, TaskHierarchy) and isinstance(source, TaskLevel):
                target.remove(source)
                target.insert(row, source)

            elif isinstance(target, TaskHierarchy) and isinstance(source, TaskItem):
                source.parent.proxy.remove(source)
                if len(source.parent) == 0:
                    target.remove(source.parent)
                target.insert_task(row, source)

            elif isinstance(target, TaskLevel) and isinstance(source, TaskLevel):
                target.parent.remove(source)
                target.parent.insert(target.row, source)
            elif isinstance(target, TaskLevel) and isinstance(source, TaskItem):
                source.parent.proxy.remove(source)
                if len(source.parent) == 0:
                    source.parent.parent.remove(source.parent)
                target.proxy.append(source)
            else:
                self.layoutChanged.emit()
                return False

        self.layoutChanged.emit()

        return True

    def add_task(self, task: Task, name: str):
        t = TaskItem(task, None)
        t.additional_data["name"] = name
        self.task_hierarchy.append_task(t)
        self.layoutChanged.emit()

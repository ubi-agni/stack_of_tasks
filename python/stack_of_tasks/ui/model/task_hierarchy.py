from __future__ import annotations

from typing import Any, Iterable, List, Optional, Sized, Union

import traits.api as ta
import traits.trait_list_object as tal
from PyQt5.QtCore import QAbstractItemModel, QMimeData, QModelIndex, Qt

from stack_of_tasks.tasks.Task import Task
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy
from stack_of_tasks.ui.traits_mapping import get_editable_trait_names

from . import RawDataRole


class MoveMimeData(QMimeData):
    mimeType = "application/ModelIndexMime"

    def __init__(self, selected_indices) -> None:
        super().__init__()
        self.selected_indices = selected_indices

    def formats(self) -> List[str]:
        return [MoveMimeData.mimeType]


class IndexProxy:
    def __init__(self, task: Task) -> None:
        self.task = task
        self.names = get_editable_trait_names(task)
        self.rows = len(self.names)


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

        parent_item = parent_item = (
            parent.internalPointer() if parent.isValid() else self.task_hierarchy
        )

        if isinstance(parent_item, Sized):
            return len(parent_item)

        if isinstance(parent_item, Task):
            if hasattr(parent_item, "_model_proxy"):
                return getattr(parent_item, "_model_proxy").rows

            mp = IndexProxy(parent_item)
            setattr(parent_item, "_model_proxy", mp)
            return mp.rows

        return 0

    def columnCount(self, parent: QModelIndex = None) -> int:
        return 2

    def index(self, row: int, column: int, parent: QModelIndex = None) -> QModelIndex:
        parent_item: Sized = self.task_hierarchy.levels

        if parent.isValid():
            parent_item = parent.internalPointer()

        if (child := self._get_child(parent_item, row, column)) is not None:
            return self.createIndex(row, column, child)

        return QModelIndex()

    def _get_child(self, item, row: int, column: int):
        if isinstance(item, Task):
            if hasattr(item, "_model_proxy"):
                return getattr(item, "_model_proxy")
            else:
                mp = IndexProxy(item)
                setattr(item, "_model_proxy", mp)
                return mp

        if row < len(item) and column == 0:
            if isinstance(item, TaskHierarchy):
                return item.levels[row]

            elif isinstance(item, list):
                return item[row]

        return None

    def parent(self, index: QModelIndex):
        if index.isValid():
            item = index.internalPointer()

            if item in self.task_hierarchy.levels:
                return QModelIndex()

            if isinstance(item, IndexProxy):
                return self.createIndex(0, 0, item.task)

            for i, l in enumerate(self.task_hierarchy.levels):
                if item in l:
                    return self.createIndex(i, 0, l)

        return QModelIndex()

    # data
    def flags(self, index: QModelIndex) -> Qt.ItemFlags:
        f = super().flags(index) ^ Qt.ItemIsSelectable

        obj = index.internalPointer()

        if isinstance(obj, Task):
            f |= Qt.ItemIsSelectable
            f |= Qt.ItemIsDragEnabled
        elif not isinstance(obj, IndexProxy):
            f |= Qt.ItemIsDropEnabled

        return f

    def data(self, index: QModelIndex, role: int) -> Any:
        if index.isValid():
            item = index.internalPointer()

            if role == Qt.DisplayRole:
                if isinstance(item, list):
                    i = self.task_hierarchy.levels.index(item)
                    return f"Level {i+1}"

                if isinstance(item, Task):
                    return f"{item.name}"

                if isinstance(item, IndexProxy):
                    data_name = item.names[index.row()]
                    if index.column() == 0:
                        return f"{data_name}"
                    else:
                        return f"{getattr(item.task, data_name)}"

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

        # Target is same level as source
        if isinstance(target, list) and all(
            source in target for source in data.selected_indices
        ):
            return False

        self.layoutAboutToBeChanged.emit()

        for source in data.selected_indices:
            # move levels around
            if isinstance(target, TaskHierarchy) and isinstance(source, list):
                target.levels.remove(source)
                target.levels.insert(row, source)

            # drop task somewere in task hierarchy
            elif isinstance(target, TaskHierarchy) and isinstance(source, Task):
                for l in target.levels:
                    if source in l:
                        l.remove(source)
                        if len(l) == 0:  # remove empty level
                            target.levels.remove(l)
                        break

                target.levels.insert(row, [source])  # insert task in new level

            elif isinstance(target, list) and isinstance(source, list):
                # drop level on another

                self.task_hierarchy.levels.remove(source)
                self.task_hierarchy.levels.insert(target.row, source)

            elif isinstance(target, list) and isinstance(source, Task):
                # task is dropped on list

                for l in self.task_hierarchy.levels:
                    if source in l:
                        l.remove(source)
                        if len(l) == 0:  # remove empty level
                            self.task_hierarchy.levels.remove(l)
                        break

                target.append(source)
            else:
                self.layoutChanged.emit()
                return False

        self.layoutChanged.emit()

        return True

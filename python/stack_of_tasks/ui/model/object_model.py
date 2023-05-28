from __future__ import annotations

from typing import Callable, Generic, List, Optional, Type, TypeVar
from typing_extensions import Self

from PyQt5.QtCore import QAbstractListModel, QModelIndex, QObject, QSortFilterProxyModel, Qt

from stack_of_tasks.ui.model import RawDataRole
from stack_of_tasks.utils.class_register import Register

ObjectType = TypeVar("ObjectType")


def display_cls_name(obj: object) -> str:
    if isinstance(obj, type):
        return obj.__name__

    else:
        return obj.__class__.__name__


class ObjectModel(Generic[ObjectType], QAbstractListModel):
    def __init__(
        self,
        data: Optional[List[ObjectType]] = None,
        disp_func: Callable[[ObjectType], str] = display_cls_name,
        parent=None,
    ) -> None:
        super().__init__(parent)

        self._objs: List[ObjectType] = [] if data is None else data
        self._disp_func = disp_func

    def rowOf(self, obj: ObjectType) -> int:
        return self._objs.index(obj)

    def rowCount(self, parent=None) -> int:
        return len(self._objs)

    def set_data_list(self, lst: List):
        self.beginResetModel()
        self._objs = lst
        self.endResetModel()

    def get_display_string(self, index: QModelIndex) -> str:
        if self._disp_func is not None:
            return self._disp_func(self._objs[index.row()])

        return str(self._objs[index.row()])

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole) -> str | ObjectType | None:
        if role == Qt.DisplayRole:
            return self.get_display_string(index)
        elif role == RawDataRole:
            return self._objs[index.row()]

    def item_changed(self, idx: int | slice):
        if isinstance(idx, int):
            start = self.index(idx, 0)
            end = start
        else:
            start = self.index(idx.start, 0)
            end = self.index(idx.stop, 0)
        self.dataChanged.emit(start, end)

    def items_inserted(self, idx: int | slice):
        if isinstance(idx, int):
            start = idx
            end = start
        else:
            start = idx.start
            end = idx.stop

        self.beginInsertRows(QModelIndex(), start, end)
        self.endInsertRows()

    def items_removed(self, idx: int | slice):
        if isinstance(idx, int):
            start = idx
            end = start
        else:
            start = idx.start
            end = idx.stop
        self.beginRemoveRows(QModelIndex(), start, end)
        self.endRemoveRows()

    @classmethod
    def from_class_register(cls: Type[Self], register: Register) -> Self:
        return cls(register.concrete_classes)


class FilterObjectModel(QSortFilterProxyModel):
    def __init__(
        self,
        source: ObjectModel,
        filter_type: Type,
        filter_subclass=False,
        parent: QObject | None = ...,
    ) -> None:
        super().__init__(parent)
        self.setSourceModel(source)
        self._filter = filter_type
        self._subclass = filter_subclass

    def setSourceModel(self, sourceModel: ObjectModel) -> None:
        return super().setSourceModel(sourceModel)

    def filterAcceptsRow(self, source_row: int, source_parent: QModelIndex) -> bool:
        if self._subclass:
            return issubclass(source_parent.child(source_row).data(RawDataRole), self._filter)
        else:
            return isinstance(source_parent.child(source_row).data(RawDataRole), self._filter)

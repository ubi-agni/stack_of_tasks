from __future__ import annotations

from typing import Generic, Iterable, Type, TypeVar
from typing_extensions import Self

from PyQt5.QtCore import QModelIndex, QObject, QSortFilterProxyModel
from PyQt5.QtGui import QStandardItemModel

from stack_of_tasks.ui.property_tree.base import RawDataItem, RawDataRole
from stack_of_tasks.ui.utils.class_register import Register

ObjectType = TypeVar("ObjectType")


class ObjectModel(Generic[ObjectType], QStandardItemModel):
    def __init__(self, data: Iterable[ObjectType] = None):
        super().__init__()

        if data is not None:
            self.extend(data)

    def find(self, value):
        for i in range(self.rowCount()):
            if self.data(self.index(i, 0), RawDataRole) == value:
                return i

    @classmethod
    def from_class_register(cls: Type[Self], register: Register) -> Self:
        return cls(register.concrete_classes)

    def append(self, obj: ObjectType):
        self.appendRow(RawDataItem(obj))

    def extend(self, objs: list[ObjectType]):
        for obj in objs:
            self.appendRow(RawDataItem(obj))


class FilterObjectModel(
    QSortFilterProxyModel
):  # TODO write filter model for filtering accepted values during input (e.g. remove cyclic offset-frames)
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
        pass
        # if self._subclass:
        #    return issubclass(source_parent.child(source_row).data(RawDataRole), self._filter)
        # else:
        #    return isinstance(source_parent.child(source_row).data(RawDataRole), self._filter)

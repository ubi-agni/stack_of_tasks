#!/usr/bin/env python3
from __future__ import annotations

import enum

from typing import Any, Generic, TypeVar

import traits.api as ta
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QStandardItem

from stack_of_tasks.ui import RawDataRole

_DataType = TypeVar("_DataType")


class BaseItem(QStandardItem):
    def raw_data(self):
        return None

    def data(self, role: int = Qt.DisplayRole) -> Any:
        if role == RawDataRole:
            return self.raw_data()

        return super().data(role)


class PlaceholderItem(QStandardItem):
    def __init__(self):
        super().__init__()

        self.setEnabled(False)
        self.setDropEnabled(False)


class RawDataItem(Generic[_DataType], BaseItem):
    def __init__(self, obj: _DataType):
        super().__init__()
        self._obj: _DataType = obj
        if isinstance(self._obj, ta.HasTraits) and hasattr(self._obj, "name"):
            print("Adding observer for ", self, obj, obj.name)
            self._obj.observe(self._nameChanged, "name")

    def __del__(self):
        if isinstance(self._obj, ta.HasTraits) and hasattr(self._obj, "name"):
            print("Removing observer for ", self._obj, self._obj.name)
            self._obj.observe(self._nameChanged, "name", remove=True)

    def _nameChanged(self, event):
        print("name changed", event.new)
        self.emitDataChanged()

    def raw_data(self):
        return self._obj

    def data(self, role: int = Qt.DisplayRole) -> Any:
        if role in [Qt.DisplayRole, Qt.EditRole]:  # return name of object
            obj = self._obj

            if isinstance(obj, enum.Enum):
                return obj.name

            elif isinstance(obj, ta.HasTraits):
                if name := getattr(obj, "name", ""):
                    return name
                else:
                    return obj.__class__.__name__
            elif isinstance(obj, type):
                return obj.__name__
            else:
                return str(obj)

        return super().data(role)

    def setData(self, value: Any, role: int) -> None:
        if role == Qt.EditRole:  # set name of object
            obj = self._obj
            if hasattr(obj, "name"):
                obj.name = value
        elif role == RawDataRole:
            setattr(self._obj, self._attr_name, value)
        else:
            return super().setData(value, role)

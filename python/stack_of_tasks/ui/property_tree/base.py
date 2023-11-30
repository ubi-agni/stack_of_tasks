#!/usr/bin/env python3
from __future__ import annotations

import enum

from typing import Any, Generic, TypeVar

import traits.api as ta
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QStandardItem

from stack_of_tasks.ui import DISPLAY_STRING_ATTR, RawDataRole

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


class RawDatatItem(Generic[_DataType], BaseItem):
    def __init__(self, obj: _DataType):
        super().__init__()

        self._obj: _DataType = obj

    def raw_data(self):
        return self._obj

    def data(self, role: int = Qt.DisplayRole) -> Any:
        if role == Qt.DisplayRole:
            data = self.raw_data()

            if isinstance(data, enum.Enum):
                return data.name

            elif isinstance(data, ta.HasTraits):
                if len(name := getattr(data, DISPLAY_STRING_ATTR, "")) > 0:
                    return name
                elif len(name := getattr(data, "name", "")) > 0:
                    return name
                else:
                    return data.__class__.__name__
            elif isinstance(data, type):
                return data.__name__
            else:
                return str(data)

        return super().data(role)

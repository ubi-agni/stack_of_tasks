#!/usr/bin/env python3
from __future__ import annotations

from enum import Enum

import typing
from typing import Any

import PyQt5.QtWidgets as QtW
import traits.api as ta
from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QStandardItem
from PyQt5.QtWidgets import QWidget

from stack_of_tasks.ui import DISPLAY_STRING_ATTR

StandardItemRole = Qt.UserRole + 15


class HasTraitsDelegate(QtW.QStyledItemDelegate):
    def displayText(self, value: typing.Any, locale: QtCore.QLocale) -> str:
        if isinstance(value, Enum):
            return value.name

        if isinstance(value, ta.HasTraits):
            if (
                DISPLAY_STRING_ATTR in value.trait_names()
                and len(text := getattr(value, DISPLAY_STRING_ATTR)) > 0
            ):
                return text
            else:
                return type(value).__name__

        return str(value)

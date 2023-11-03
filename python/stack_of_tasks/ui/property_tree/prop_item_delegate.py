#!/usr/bin/env python3
from __future__ import annotations

import typing

import numpy as np
import traits.api as ta
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import QModelIndex, QPersistentModelIndex, QSize, Qt
from PyQt5.QtGui import QStandardItem
from PyQt5.QtWidgets import QStyle, QStyleOptionViewItem, QWidget

from stack_of_tasks.ui import ItemRole, MappingEntryRole, RawDataRole, TraitRole
from stack_of_tasks.ui.traits_mapping.bindings import get_user_property, set_user_property
from stack_of_tasks.ui.traits_mapping.painter_entry.data_painter import MatrixPainter
from stack_of_tasks.ui.traits_mapping.ui_mapping import MappingEntry

from .has_traits_delegate import HasTraitsDelegate


class PropItemDelegate(HasTraitsDelegate):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._current_editor = None
        self._current_editor_index = None

    def sizeHint(self, option: QStyleOptionViewItem, index: QModelIndex) -> QtCore.QSize:
        if self._current_editor_index is not None and self._current_editor_index == index:
            return self._current_editor.sizeHint()

        me: MappingEntry = index.data(MappingEntryRole)
        if me is not None:
            data = index.data(RawDataRole)
            size_hint: QSize = me.painter.size_hint(option, data)

            if size_hint.isValid():
                return size_hint

        return super().sizeHint(option, index)

    def createEditor(
        self, parent: QWidget, option: QStyleOptionViewItem, index: QModelIndex
    ) -> QWidget:
        me: MappingEntry = index.data(MappingEntryRole)
        trait: ta.CTrait = index.data(TraitRole)

        if me is not None:
            widget = me.widget(parent)
            if option.state == QStyle.State_Selected:
                widget.setBackgroundRole(option.palette.Highlight)
            me.setup_function(trait, widget)

            self._current_editor = widget
            self._current_editor_index = QPersistentModelIndex(index)
            self.sizeHintChanged.emit(index)

            return widget
        else:
            return super().createEditor(parent, option, index)

    def destroyEditor(self, editor: QWidget, index: QModelIndex) -> None:
        self._current_editor = None
        self._current_editor_index = None

        super().destroyEditor(editor, index)
        self.sizeHintChanged.emit(index)

    def setEditorData(self, editor: QWidget, index: QModelIndex) -> None:
        value = index.data(RawDataRole)
        print("set editor data", value)
        set_user_property(editor, value)

    def setModelData(
        self, editor: QWidget, model: QtCore.QAbstractItemModel, index: QModelIndex
    ) -> None:
        val = get_user_property(editor)
        model.setData(index, val, RawDataRole)

    def paint(
        self, painter: QtGui.QPainter, option: QStyleOptionViewItem, index: QModelIndex
    ) -> None:
        me: MappingEntry = index.data(MappingEntryRole)
        if me is not None and me.painter.has_Painter:
            data = index.data(RawDataRole)
            me.painter.paint(painter, option, data)

        else:
            super().paint(painter, option, index)

#!/usr/bin/env python3
from __future__ import annotations

import typing

import traits.api as ta
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import QModelIndex, QPersistentModelIndex, QSize, Qt
from PyQt5.QtWidgets import QStyle, QStyledItemDelegate, QStyleOptionViewItem, QWidget

from stack_of_tasks.ui import MappingEntryRole, RawDataRole, TraitRole
from stack_of_tasks.ui.traits_mapping.bindings import get_user_property, set_user_property
from stack_of_tasks.ui.traits_mapping.ui_mapping import MappingEntry


class PropItemDelegate(QStyledItemDelegate):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._current_editor = None
        self._current_editor_index = None
        self._edit_role = Qt.EditRole

    def sizeHint(self, option: QStyleOptionViewItem, index: QModelIndex) -> QtCore.QSize:
        if self._current_editor_index is not None and self._current_editor_index == index:
            return self._current_editor.sizeHint()

        me: MappingEntry = index.data(MappingEntryRole)
        if me is not None and me.painter is not None:
            data = index.data(RawDataRole)
            return me.painter.size_hint(option, data)

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
            # column 1 changes raw data value, column 0 changes name
            self._edit_role = RawDataRole if index.column() == 1 else Qt.EditRole
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
        value = index.data(self._edit_role)
        set_user_property(editor, value)

    def setModelData(
        self, editor: QWidget, model: QtCore.QAbstractItemModel, index: QModelIndex
    ) -> None:
        val = get_user_property(editor)
        model.setData(index, val, self._edit_role)

    def paint(
        self, painter: QtGui.QPainter, option: QStyleOptionViewItem, index: QModelIndex
    ) -> None:
        me: MappingEntry = index.data(MappingEntryRole)
        if me is not None and me.painter is not None:
            data = index.data(RawDataRole)
            me.painter.paint(painter, option, data)

        else:
            super().paint(painter, option, index)

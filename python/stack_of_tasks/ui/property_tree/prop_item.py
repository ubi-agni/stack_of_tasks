#!/usr/bin/env python3
from __future__ import annotations

import enum

import typing
from typing import Any, Generic, TypeVar

import traits.api as ta
from PyQt5.QtCore import Qt

from stack_of_tasks.ui import DISPLAY_STRING_ATTR, MappingEntryRole, RawDataRole, TraitRole
from stack_of_tasks.ui.traits_mapping import is_editable_trait
from stack_of_tasks.ui.traits_mapping.ui_mapping import Mapping, MappingEntry

from .base import BaseItem, PlaceholderItem, RawDataItem


class LevelItem(BaseItem):
    def __init__(self):
        super().__init__()

    def data(self, role: int = Qt.DisplayRole) -> Any:
        if role == Qt.DisplayRole:
            return f"Level {self.row() + 1}"

        return super().data(role)


class TraitTreeBase(BaseItem):
    def _setup_children(self, obj: ta.HasTraits):
        self.removeRows(0, self.rowCount())

        for name in obj.visible_traits():
            self._add_child(obj, name)

    def _add_child(self, obj, name):
        if (me := Mapping.find_entry(obj.trait(name))) is not None:
            attr_name = AttrNameItem(obj, name)
            attr_value = AttrValueItem(obj, name)
            self.appendRow([attr_name, attr_value])

    def _remove_child(self, name):
        for i in self.rowCount():
            if (child := self.child(i, 0)).text() == name:
                self.removeRow(i)
                break


class TaskItem(RawDataItem, TraitTreeBase):
    def __init__(self, obj: ta.HasTraits):
        super().__init__(obj)
        self._setup_children(self._obj)
        obj.observe(self._remove_child, "trait_removed")


class AttrNameItem(TraitTreeBase):
    def __init__(self, obj: ta.HasTraits, attr_name: str):
        super().__init__()

        self.setSelectable(False)
        self.setEditable(False)

        self.setText(attr_name)

        data = getattr(obj, attr_name)
        if isinstance(data, ta.HasTraits):
            obj.observe(self._attr_change_cb, attr_name)
            self._setup_children(data)

    def _attr_change_cb(self, evt):
        self._setup_children(evt.new)


class AttrValueItem(RawDataItem):
    def __init__(self, obj: ta.HasTraits, attr_name: str):
        super().__init__(obj)
        self._attr_name = attr_name
        self._trait: ta.CTrait = obj.trait(attr_name)
        self._trait_map_entry: MappingEntry = Mapping.find_entry(self._trait)

        self.setSelectable(False)
        self.setEditable(is_editable_trait(attr_name, self._trait))
        self.setDragEnabled(False)
        self.setDropEnabled(False)

        obj.observe(self._data_changed, attr_name)

    def _data_changed(self, obj):
        self.emitDataChanged()

    def raw_data(self):
        return getattr(self._obj, self._attr_name)

    def setData(self, data: Any, role: int = Qt.DisplayRole):
        if role == RawDataRole:
            setattr(self._obj, self._attr_name, data)
        super().setData(data, role)
        self.emitDataChanged()

    def data(self, role: int = Qt.DisplayRole) -> Any:
        if role == MappingEntryRole:
            return self._trait_map_entry

        elif role == TraitRole:
            return self._trait

        return super().data(role)

from __future__ import annotations

from typing import Generic, List, Type, TypeVar

import traits.api as ta
import traits.trait_types as tt
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QWidget
from traits.trait_numeric import Array

from stack_of_tasks.ui.model.object_model import ObjectModel
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.widgets.matrix_view import MatrixWidget
from stack_of_tasks.ui.widgets.object_dropbown import AddableObjectDropdown, ObjectDropdown
from stack_of_tasks.ui.widgets.range import Range

from .painter_entry.data_painter import Delegate_Painter, MatrixPainter


class Mapping:
    mappings: List[MappingEntry] = []

    @classmethod
    def find_entry(cls, trait: ta.CTrait):
        if getattr(trait, "enum_selection") is not None:  # temp. workaround
            return Enum

        for m in cls.mappings:
            if m.matches(trait):
                return m

    @classmethod
    def find_widget(cls, trait: ta.CTrait):
        if (e := cls.find_entry(trait)) is not None:
            return e.widget, e.setup_function


MappingType = TypeVar("MappingType")


class MappingEntry(Generic[MappingType]):
    traits: List[ta.CTrait]

    painter: Type[Delegate_Painter | None] = None
    widget: Type[QWidget] = None

    def __init_subclass__(cls, *args, **kwargs) -> None:
        super().__init_subclass__(*args, **kwargs)
        Mapping.mappings.append(cls)

    @classmethod
    def matches(cls, trait: ta.CTrait) -> bool:
        return any(isinstance(trait.trait_type, x) for x in cls.traits)

    @classmethod
    def setup_function(cls, trait: ta.CTrait, widget: QWidget):
        pass


class String(MappingEntry):
    traits = [tt.String, tt.BaseStr, tt.BaseCStr]
    widget = QtWidgets.QLineEdit


class Float(MappingEntry):
    traits = [tt.BaseFloat, tt.BaseCFloat]

    widget = QtWidgets.QDoubleSpinBox


class RangeEntry(MappingEntry):
    traits = [tt.BaseRange]
    widget = Range

    @classmethod
    def setup_function(cls, trait: ta.CTrait, widget: Range):
        rng: ta.Range = trait.trait_type

        widget.setMinimum(rng._low)
        widget.setMaximum(rng._high)
        widget.setExcludeHigh(rng._exclude_high)
        widget.setExcludeLow(rng._exclude_low)
        widget.setSingleStep(0.01)


class Selection(MappingEntry):
    traits = [tt.Instance]
    widget = AddableObjectDropdown

    @classmethod
    def setup_function(cls, trait: ta.CTrait, widget: AddableObjectDropdown):
        t: tt.Instance = trait.trait_type
        key = t.model_key if t.model_key is not None else t.klass

        object_model = ModelMapping.get_mapping(key)
        cls_model = ModelMapping.get_mapping(ClassKey(key))
        widget.cls_model = cls_model

        if object_model is not None:
            widget.setModel(object_model)


class Enum(MappingEntry):
    traits = [tt.BaseEnum]
    widget = ObjectDropdown

    @classmethod
    def setup_function(cls, trait: ta.CTrait, widget: ObjectDropdown):
        if getattr(trait, "enum_selection") is not None:
            widget.setModel(getattr(trait, "enum_selection"))

        else:
            widget.setModel(ObjectModel(data=trait.trait_type.values))


class Matrix(MappingEntry):
    traits = [Array]
    widget = MatrixWidget
    painter = MatrixPainter

    @classmethod
    def matches(cls, trait: ta.CTrait) -> bool:
        return isinstance(trait.trait_type, Array)

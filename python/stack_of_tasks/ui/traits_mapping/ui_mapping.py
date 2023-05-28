from __future__ import annotations

from typing import Callable, Generic, List, Type, TypedDict, TypeVar, Union

import PyQt5.QtWidgets as QTW
import traits.api as ta
import traits.trait_base as tb
import traits.trait_types as tt

from stack_of_tasks.ui.model_mapping import ModelMapping
from stack_of_tasks.ui.widgets.matrix_view import MatrixView, NumpyTableModel
from stack_of_tasks.ui.widgets.object_dropbown import ObjectDropdown, ObjectModel
from stack_of_tasks.ui.widgets.range import Range


class Mapping:
    mappings: List[MappingEntry] = []

    @classmethod
    def find_widget(cls, trait: ta.CTrait):
        if trait.selection_model is not None:
            return NoneInstanceSelection.widget(trait), NoneInstanceSelection.setup_function

        for m in cls.mappings:
            if m.matches(trait):
                return m.widget(trait), m.setup_function


MappingType = TypeVar("MappingType")


class MappingEntry(Generic[MappingType]):
    traits: List[ta.CTrait]

    def __init_subclass__(cls, **kwargs) -> None:
        super().__init_subclass__(**kwargs)
        Mapping.mappings.append(cls)

    @classmethod
    def matches(cls, trait: ta.CTrait) -> bool:
        return any(isinstance(trait.trait_type, x) for x in cls.traits)

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[MappingType]:
        pass

    @classmethod
    def setup_function(cls, trait: ta.CTrait, trait_name: str, widget: QTW, value=None):
        pass


class String(MappingEntry):
    traits = [tt.String, tt.BaseStr, tt.BaseCStr]

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[QTW.QWidget]:
        return QTW.QLineEdit

    @classmethod
    def setup_function(
        cls, trait: ta.CTrait, trait_name: str, widget: QTW.QLineEdit, value=None
    ):
        if value is None:
            value = trait.default

        widget.setText(value)


class Float(MappingEntry):
    traits = [tt.BaseFloat, tt.BaseCFloat]

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[QTW.QWidget]:
        return QTW.QDoubleSpinBox

    @classmethod
    def setup_function(
        cls, trait: ta.CTrait, trait_name: str, widget: QTW.QDoubleSpinBox, value=None
    ):
        if value is None:
            value = trait.default

        widget.setValue(value)


class RangeEntry(MappingEntry):
    traits = [tt.BaseRange]

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[QTW.QWidget]:
        return Range

    @classmethod
    def setup_function(cls, trait: ta.CTrait, trait_name: str, widget: Range, value=None):
        rng: ta.Range = trait.trait_type

        if value is None:
            value = trait.default

        widget.setValue(value)

        widget.setMinimum(rng._low)
        widget.setMaximum(rng._high)
        widget.setExcludeHigh(rng._exclude_high)
        widget.setExcludeLow(rng._exclude_low)
        widget.setSingleStep(0.01)


class Selection(MappingEntry):
    traits = [tt.Instance]

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[ObjectDropdown]:
        return ObjectDropdown

    @classmethod
    def setup_function(
        cls, trait: ta.CTrait, trait_name: str, widget: ObjectDropdown, value=None
    ):
        t: tt.Instance = trait.trait_type
        key = t.model_key if t.model_key is not None else t.klass
        refs_model: ObjectModel = ModelMapping.get_mapping(key)

        if refs_model is not None:
            widget.setModel(refs_model)

            if value is not None:
                widget.current_object = value

        else:
            if value is None:
                value = trait.default
                widget.current_object = value
                widget.setModel(ObjectModel(data=[value]))


class NoneInstanceSelection(Selection):
    traits = []

    @classmethod
    def matches(cls, trait: ta.CTrait) -> bool:
        return False

    @classmethod
    def setup_function(
        cls, trait: ta.CTrait, trait_name: str, widget: ObjectDropdown, value=None
    ):
        widget.setModel(trait.selection_model)

        if value is not None:
            if isinstance(value, str) and len(value) == 0:
                return

            widget.current_object = value


class Enum(MappingEntry):
    traits = [tt.BaseEnum]

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[ObjectDropdown]:
        return ObjectDropdown

    @classmethod
    def setup_function(
        cls, trait: ta.CTrait, trait_name: str, widget: ObjectDropdown, value=None
    ):
        widget.setModel(ObjectModel(data=trait.trait_type.values))
        if value is None:
            widget.current_object = trait.default
        else:
            widget.current_object = value


from traits.trait_numeric import Array


class Matrix(MappingEntry):
    traits = [Array]

    @classmethod
    def matches(cls, trait: ta.CTrait) -> bool:
        return isinstance(trait.trait_type, Array)

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[MatrixView]:
        return MatrixView

    @classmethod
    def setup_function(
        cls, trait: ta.CTrait, trait_name: str, widget: MatrixView, value=None
    ):
        model = NumpyTableModel()

        if value is None:
            value = trait.trait_type.default_value

        model.setMatrix(value)
        widget.setModel(model)


class ReadOnly(MappingEntry):
    traits = [Array]

    @classmethod
    def matches(cls, trait: ta.CTrait) -> bool:
        return type(trait.trait_type) is type(ta.ReadOnly) and trait.default is tb.Undefined

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[MatrixView]:
        return None

    @staticmethod
    def setup_function(obj: ta.HasTraits, trait_name: str, widget: MatrixView):
        pass

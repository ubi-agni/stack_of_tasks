from __future__ import annotations

from typing import Callable, Generic, List, Type, TypedDict, TypeVar, Union

import PyQt5.QtWidgets as QTW
import traits.api as ta
import traits.trait_types as tt
from stack_of_tasks_ui.traits_mapping.custom_widgets.matrix_view import (
    MatrixView,
    NumpyTableModel,
)
from stack_of_tasks_ui.traits_mapping.custom_widgets.object_dropbown import (
    ObjectDropdown,
    ObjectModel,
)
from stack_of_tasks_ui.traits_mapping.custom_widgets.range import Range


class Mapping:
    mappings: List[MappingEntry] = []

    @classmethod
    def find_widget(cls, trait: ta.CTrait):
        for m in cls.mappings:
            if m.matches(trait):
                return m.widget(trait), m.setup_function


MappingType = TypeVar("MappingType")


class MappingEntry(Generic[MappingType]):
    traits: List[ta.CTrait]

    def __init_subclass__(cls) -> None:
        Mapping.mappings.append(cls)

    @classmethod
    def matches(cls, trait: ta.CTrait) -> bool:
        return any(isinstance(trait.trait_type, x) for x in cls.traits)

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[MappingType]:
        pass

    @classmethod
    def setup_function(cls, obj: ta.HasTraits, trait_name: str, widget: MappingType):
        pass


class String(MappingEntry):
    traits = [tt.String, tt.BaseStr, tt.BaseCStr]

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[QTW.QWidget]:
        return QTW.QLineEdit


class RangeEntry(MappingEntry):
    traits = [tt.BaseRange]

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[QTW.QWidget]:
        return Range

    @staticmethod
    def setup_function(obj: ta.HasTraits, trait_name: str, widget: Range):
        rng: ta.Range = obj.trait(trait_name).trait_type

        widget.setValue(getattr(obj, trait_name))
        widget.setMinimum(rng._low)
        widget.setMaximum(rng._high)
        widget.setExcludeHigh(rng._exclude_high)
        widget.setExcludeLow(rng._exclude_low)
        widget.setSingleStep(0.01)


class Selection(MappingEntry):
    traits = [tt.Instance]

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[QTW.QDoubleSpinBox]:
        return ObjectDropdown


class Enum(MappingEntry):
    traits = [tt.BaseEnum]

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[ObjectDropdown]:
        return ObjectDropdown

    @staticmethod
    def setup_function(obj: ta.HasTraits, trait_name: str, widget: ObjectDropdown):
        widget.setModel(ObjectModel(data=obj.trait(trait_name).trait_type.values))
        widget.current_object = getattr(obj, trait_name)


from traits.trait_numeric import Array


class Matrix(MappingEntry):
    traits = [Array]

    @classmethod
    def matches(cls, trait: ta.CTrait) -> bool:
        print(isinstance(trait.trait_type, Array))
        return isinstance(trait.trait_type, Array)

    @classmethod
    def widget(cls, trait: ta.CTrait) -> Type[MatrixView]:
        return MatrixView

    @staticmethod
    def setup_function(obj: ta.HasTraits, trait_name: str, widget: MatrixView):
        # print(getattr(obj, trait_name))
        model = NumpyTableModel()
        model.setMatrix(getattr(obj, trait_name))
        widget.setModel(model)

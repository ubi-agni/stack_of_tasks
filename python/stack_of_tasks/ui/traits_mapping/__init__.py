from __future__ import annotations

import typing

import PyQt5.QtWidgets as QtW
import traits.api as ta
from PyQt5.QtCore import pyqtProperty
from PyQt5.QtWidgets import QFormLayout, QWidget

from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.solver.OSQPSolver import OSQPSolver

SOLVER = [InverseJacobianSolver, OSQPSolver]
from typing import Any, Type

import traits.api as ta
import traits.trait_types as tt
from traits.trait_type import _read_only, _write_only

from stack_of_tasks.ui.traits_mapping.bindings import (
    get_user_property,
    set_user_property,
    trait_widget_binding,
)
from stack_of_tasks.ui.traits_mapping.ui_mapping import Mapping


def is_editable_trait(name: str, trait: ta.CTrait) -> bool:
    # name starts with underscore?
    if name.startswith("_"):
        return False

    # is property?
    if trait.is_property:
        if trait.property_fields[1] is _read_only or trait.property_fields[0] is _write_only:
            return False
        return True

    return True


def get_editable_trait_names(obj: ta.HasTraits):
    trait_names = []
    for trait_name in obj.editable_traits():
        trait: ta.CTrait = obj.base_trait(trait_name)
        if is_editable_trait(trait_name, trait):
            trait_names.append(trait_name)

    return trait_names


def get_init_arg_trait_names(cls: type[ta.HasTraits]):
    cls_trait_names = cls.class_visible_traits()
    cls_trts = cls.class_traits()

    trait_names = []
    for name in cls_trait_names:
        if is_editable_trait(name, cls_trts[name]):
            trait_names.append(name)
    return trait_names


def is_valid_value(obj: ta.HasTraits | ta.MetaHasTraits, name: str, val: Any) -> bool:
    if isinstance(obj, type):
        trt = obj.class_traits()[name]
    else:
        trt = obj.base_trait(name)

    r = True
    try:
        trt.validate(obj, name, val)
    except ta.TraitError:
        r = False
    return r


from stack_of_tasks.ui.model_mapping import InjectionArg


class NewInstanceWidget(QWidget):
    def __init__(self, parent=None, cls=None) -> None:
        super().__init__(parent)

        self.cls = cls
        self.args = {}

        self.fl = QFormLayout()
        self.setLayout(self.fl)

    def _setup_widgets(self):
        self.args.clear()

        while self.fl.rowCount() > 0:
            self.fl.removeRow(0)

        cls_trs = self.cls.class_traits()
        for name in get_init_arg_trait_names(self.cls):
            tr = cls_trs[name]

            if tr.injected:
                self.args[name] = InjectionArg()
                continue

            w = Mapping.find_widget(tr)

            if w is not None:
                widget_cls, setup = w
                widget = widget_cls()
                setup(tr, name, widget, None)

            else:
                continue

            self.fl.addRow(name, widget)
            self.args[name] = widget

    def get_arguments(self) -> dict["str", Any]:
        r = {}
        for k, v in self.args.items():
            if isinstance(v, InjectionArg):
                r[k] = v
            else:
                r[k] = get_user_property(v)
                # print(f"arg_name: {k}, {get_user_property(v)}, {v}")

        return r


class HasTraitsFormLayout(QFormLayout):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)

    def setHastTrait(self, inst: ta.HasTraits):
        for name in get_editable_trait_names(inst):
            trait = inst.trait(name)
            if trait.injected:
                continue
            match = Mapping.find_widget(trait)

            if match is not None:
                widget_cls, setup_function = match
                widget = widget_cls()
                setup_function(trait, name, widget, getattr(inst, name))

                trait_widget_binding(inst, name, widget)

                self.addRow(name, widget)

    def clear_widget(self):
        while self.rowCount() > 0:
            self.removeRow(0)


class HasTraitPropertyMixin(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.trait_form_layout = HasTraitsFormLayout()
        self.setLayout(self.trait_form_layout)

    def set_trait_object(self, obj):
        self.trait_form_layout.clear_widget()
        self.trait_form_layout.setHastTrait(obj)

    trait_object = pyqtProperty(ta.HasTraits, fset=set_trait_object)


class HasTraitGroupBox(QtW.QGroupBox, HasTraitPropertyMixin):
    pass

from __future__ import annotations

from typing import Any

import traits.api as ta
from PyQt5.QtCore import pyqtProperty
from PyQt5.QtWidgets import QFormLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

from stack_of_tasks.ui.model_mapping import InjectionArg
from stack_of_tasks.ui.traits_mapping import (
    get_editable_trait_names,
    get_init_arg_trait_names,
)
from stack_of_tasks.ui.traits_mapping.bindings import get_user_property, trait_widget_binding
from stack_of_tasks.ui.traits_mapping.ui_mapping import Mapping


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


class HasTraitWidget(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.trait_form_layout = HasTraitsFormLayout()
        self.setLayout(self.trait_form_layout)

    def set_trait_object(self, obj):
        self.trait_form_layout.clear_widget()
        if obj is not None:
            self.trait_form_layout.setHastTrait(obj)

    trait_object = pyqtProperty(ta.HasTraits, fset=set_trait_object)


class HasTraitGroupBox(QGroupBox, HasTraitWidget):
    pass


class EditorGroupBox(QGroupBox):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self._Vlayout = QVBoxLayout()
        self.none_selected = QLabel("No object selected.")
        self.trait_widget = HasTraitWidget()

        self._Vlayout.addWidget(self.none_selected)
        self._Vlayout.addWidget(self.trait_widget)
        self.setLayout(self._Vlayout)

        self.trait_widget.hide()

    def set_trait_object(self, obj):
        self.trait_widget.set_trait_object(obj)

        is_None = obj is None
        self.trait_widget.setVisible(not is_None)
        self.none_selected.setVisible(is_None)

    trait_object = pyqtProperty(ta.HasTraits, fset=set_trait_object)

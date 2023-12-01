from __future__ import annotations

from typing import Any

import traits.api as ta
from PyQt5.QtCore import pyqtProperty
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from stack_of_tasks.ui.model_mapping import InjectionArg
from stack_of_tasks.ui.traits_mapping import get_init_arg_trait_names, ui_mapping
from stack_of_tasks.ui.traits_mapping.bindings import (
    get_user_property,
    set_user_property,
    trait_widget_binding,
)


class NewInstanceWidget(QWidget):
    def __init__(self, parent=None, cls=None) -> None:
        super().__init__(parent)

        self.cls = cls
        self.args = {}

        self.fl = QFormLayout()
        self.setLayout(self.fl)

    def _setup_widgets(self):
        old_args = {k: get_user_property(v) for k, v in self.args.items()}
        self.args.clear()

        while self.fl.rowCount() > 0:
            self.fl.removeRow(0)

        cls_trs = self.cls.class_traits()

        for name in get_init_arg_trait_names(self.cls):
            trait: ta.CTrait = cls_trs[name]

            if trait.injected is not None:
                self.args[trait.injected] = InjectionArg()
                continue

            me = ui_mapping.Mapping.find_entry(trait)

            if me is not None:
                widget = me.widget()
                me.setup_function(trait, widget)
                value = old_args.get(name, trait.trait_type.default_value)
                set_user_property(widget, value)
                self.fl.addRow(name, widget)
                self.args[name] = widget

    def get_arguments(self) -> dict["str", Any]:
        r = {}

        for k, v in self.args.items():
            if isinstance(v, InjectionArg):
                r[k] = v
            else:
                r[k] = get_user_property(v)

        return r


class HasTraitsFormLayout(QFormLayout):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)

    def fill(self, inst: ta.HasTraits):
        for name in inst.visible_traits():
            trait = inst.trait(name)
            if trait.injected:
                continue

            me = ui_mapping.Mapping.find_entry(trait)

            if me is not None:
                widget = me.widget()
                me.setup_function(trait, widget)

                set_user_property(widget, getattr(inst, name))

                trait_widget_binding(inst, name, widget)
                self.addRow(name, widget)

    def clear(self):
        while self.rowCount() > 0:
            self.removeRow(0)


class LinkButtonWrapper(QHBoxLayout):
    def __init__(self, widget):
        super().__init__()

        self.addWidget(widget)
        self.link_button = QPushButton(icon=QIcon.fromTheme("insert-link"))
        self.link_button.setContentsMargins(0, 0, 0, 0)
        self.addWidget(self.link_button)


class HasTraitWidget(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.trait_form_layout = HasTraitsFormLayout()
        self.setLayout(self.trait_form_layout)

    def set_trait_object(self, obj):
        self.trait_form_layout.clear()
        if obj is not None:
            self.trait_form_layout.fill(obj)

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

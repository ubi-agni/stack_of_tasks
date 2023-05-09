import typing

import PyQt5.QtWidgets as QtW
import traits.api as ta
from PyQt5.QtWidgets import QFormLayout, QWidget

from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.solver.OSQPSolver import OSQPSolver

SOLVER = [InverseJacobianSolver, OSQPSolver]

import traits.api as ta

from stack_of_tasks.ui.traits_mapping.bindings import get_user_prop_name, trait_widget_binding
from stack_of_tasks.ui.traits_mapping.ui_mapping import Mapping


class HasTraitsFormLayout(QFormLayout):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)

    def setHastTrait(self, inst: ta.HasTraits):
        for name in inst.editable_traits():
            trait = inst.trait(name)

            w: typing.Type[QWidget] = Mapping.find_widget(trait)

            if w is not None:
                w, f = w
                winst = w()
                f(inst, name, winst)

                # trait_widget_binding(inst, name, winst)

                self.addRow(name, winst)

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


class HasTraitGroupBox(QtW.QGroupBox, HasTraitPropertyMixin):
    pass

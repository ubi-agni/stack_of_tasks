from PyQt5.QtWidgets import QWidget, QDoubleSpinBox
from stack_of_tasks.ui.generated.SolverSettings import Ui_SolverSettings
import typing

from stack_of_tasks.solver import InverseJacobianSolver, OSQPSolver

from stack_of_tasks.ui.model.object_model import ObjectModel, RawDataRole


class SolverTab(QWidget, Ui_SolverSettings):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setupUi(self)

        self._solver_model = ObjectModel()
        self._solver_model.add_object(OSQPSolver, name="QP-Solver: OSQP")
        self._solver_model.add_object(InverseJacobianSolver, name="Inverse Jacobian")

        self.solverClassComboBox.setModel(self._solver_model)
        self.solverClassComboBox.currentIndexChanged.connect(self._solver_class_selected)

        self._options_widgets = {}
        self._solver_class_selected()

    @property
    def solver_class(self):
        return self.solverClassComboBox.currentData(RawDataRole)

    @property
    def options(self):
        return {k: v() for k, v in self._options_widgets.items()}

    def _add_float_row(self, label, min=0.0, max=1.0, step=0.1, default=1):
        w = QDoubleSpinBox()
        w.setMinimum(min)
        w.setMaximum(max)
        w.setSingleStep(step)
        self._options_widgets[label] = w.value

        self.form_layout.addRow(label, w)

    def _solver_class_selected(self):
        cls = self.solver_class
        for i in range(1, self.form_layout.rowCount()):
            self.form_layout.removeRow(i)

        if cls is InverseJacobianSolver:
            # options are:
            # threshold: float[0,1]
            self._add_float_row("threshold")

        elif cls is OSQPSolver:
            # options are:
            self._add_float_row("rho", default=0.01)

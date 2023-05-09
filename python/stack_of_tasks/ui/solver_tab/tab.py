from PyQt5.QtWidgets import QDoubleSpinBox, QFormLayout, QVBoxLayout, QWidget

from stack_of_tasks.solver import InverseJacobianSolver, OSQPSolver
from stack_of_tasks.ui.generated.SolverSettings import Ui_SolverSettings
from stack_of_tasks.ui.model.object_model import ObjectModel, RawDataRole
from stack_of_tasks.ui.traits_mapping import HasTraitGroupBox
from stack_of_tasks.ui.traits_mapping.custom_widgets.object_dropbown import ObjectDropdown


class SolverTab(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.vl = QVBoxLayout()
        self.setLayout(self.vl)

        self.solver_class = ObjectDropdown()
        fl1 = QFormLayout()
        fl1.addRow("Solver", self.solver_class)
        self.vl.addLayout(fl1)

        self.solver_settings = HasTraitGroupBox()
        self.vl.addWidget(self.solver_settings)

        self.solver_class.currentTextChanged.connect(
            lambda title: self.solver_settings.setTitle(f"{title} - Settings")
        )

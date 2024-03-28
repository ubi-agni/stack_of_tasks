from PyQt5.QtWidgets import QAction, QFormLayout, QToolButton, QVBoxLayout, QWidget

from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.widgets.has_trait_widgets import HasTraitWidget
from stack_of_tasks.ui.widgets.object_dropbown import ObjectDropdown


class SolverSettings(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.verticalLayout = QVBoxLayout()
        self.cls_layout = QFormLayout()
        self.cls_layout.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)

        self.solverClassComboBox = ObjectDropdown()
        self.cls_layout.addRow("Class", self.solverClassComboBox)

        model = ModelMapping.get_mapping(ClassKey(Solver))

        self.verticalLayout.addLayout(self.cls_layout)

        self.edit_solver = HasTraitWidget()
        self.verticalLayout.addWidget(self.edit_solver)
        self.verticalLayout.setStretch(1, 1)

        self._set_solver_action = QAction
        self._set_button = QToolButton()

        self.setLayout(self.verticalLayout)

        self.solverClassComboBox.setModel(model)

from PyQt5 import QtWidgets

from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.widgets.has_trait_widgets import HasTraitWidget
from stack_of_tasks.ui.widgets.object_dropbown import ObjectDropdown


class SolverTab(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.cls_layout = QtWidgets.QFormLayout()
        self.cls_layout.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldsStayAtSizeHint)

        self.solverClassComboBox = ObjectDropdown()
        self.cls_layout.addRow("Class", self.solverClassComboBox)

        model = ModelMapping.get_mapping(ClassKey(Solver))

        self.verticalLayout.addLayout(self.cls_layout)
        self.edit_solver = HasTraitWidget()
        self.verticalLayout.addWidget(self.edit_solver)
        self.verticalLayout.setStretch(1, 1)
        self.setLayout(self.verticalLayout)

        self.solverClassComboBox.setModel(model)

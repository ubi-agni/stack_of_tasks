from PyQt5.QtWidgets import (
    QAction,
    QCheckBox,
    QFormLayout,
    QGroupBox,
    QLineEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from stack_of_tasks.robot_model.actuators import Actuator
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.widgets.has_trait_widgets import HasTraitWidget
from stack_of_tasks.ui.widgets.object_dropbown import ObjectDropdown


class SettingsTabWidget(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.verticalLayout = QVBoxLayout()
        self.setLayout(self.verticalLayout)

        layout = self._new_region("Settings", QFormLayout())

        self.proj_name = QLineEdit()
        layout.addRow("Project name", self.proj_name)

        self.plot_switch = QCheckBox()
        self.plot_switch.setEnabled(False)
        layout.addRow("Plot", self.plot_switch)

        layout = self._new_region("Solver (Only OSQP is working)", QVBoxLayout())

        cls_layout = QFormLayout()
        cls_layout.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)
        layout.addLayout(cls_layout)
        self.solverClassComboBox = ObjectDropdown()
        cls_layout.addRow("Class", self.solverClassComboBox)

        model = ModelMapping.get_mapping(ClassKey(Solver))
        self.solverClassComboBox.setModel(model)

        self.edit_solver = HasTraitWidget()
        layout.addWidget(self.edit_solver)

        layout = self._new_region("Actuator class", QVBoxLayout())
        actuator_layout = QFormLayout()
        actuator_layout.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)
        layout.addLayout(actuator_layout)

        self.actuatorClassComboBox = ObjectDropdown()
        actuator_layout.addRow("Class", self.actuatorClassComboBox)
        self.actuatorClassComboBox.setModel(ModelMapping.get_mapping(ClassKey(Actuator)))
        self.edit_actuator = HasTraitWidget()
        layout.addWidget(self.edit_actuator)

        self.verticalLayout.addStretch()

        self._set_solver_action = QAction
        self._set_button = QToolButton()

    def _new_region(self, name, layout):
        group = QGroupBox(name)
        group.setLayout(layout)
        self.verticalLayout.addWidget(group)
        return layout

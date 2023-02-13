from stack_of_tasks.tasks.Task import RelativeTask, TaskSoftnessType
from stack_of_tasks.ui.data_input import DataInput
from stack_of_tasks.ui.models import EnumModel, RawDataRole


class TaskDetails(DataInput):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self._name = self.add_string_row("Name")
        self._soft_type = self.add_combo_row("Softness type", EnumModel(TaskSoftnessType))

        self._refA = self.add_combo_row("Ref A")

        self._refB = self.add_combo_row("Ref B")

        self._rel_type = self.add_combo_row("Relation", EnumModel(RelativeTask.RelativeType))

        self._weight = self.add_float_row("Weight", max=10, step=0.01)

        # self.task: Task = None
        # self.nameLineEdit.textChanged.connect(self._fire_property("name"))

    #
    # self.relComboBox.currentIndexChanged.connect(
    #    self._fire_property(
    #        "rel_type", lambda _: self.relComboBox.currentData(RawDataRole)
    #    )
    # )
    #
    # self.softnessTypeComboBox.currentIndexChanged.connect(
    #    self._fire_property(
    #        "soft_type", lambda _: self.softnessTypeComboBox.currentData(RawDataRole)
    #    )
    # )
    #
    # self.refAComboBox.currentIndexChanged.connect(
    #    self._fire_property("refA", lambda _: self.refAComboBox.currentData(RawDataRole))
    # )
    #
    # self.refBComboBox.currentIndexChanged.connect(
    #    self._fire_property("refB", lambda _: self.refBComboBox.currentData(RawDataRole))
    # )
    #
    # self.weightDoubleSpinBox.valueChanged.connect(self._fire_property("weight"))

    def setModel(self, ref_model):
        self._refA.widget.setModel(ref_model)
        self._refB.widget.setModel(ref_model)

    def set_values_from_task(self, task: RelativeTask, name: str):
        self._name.widget.setText(name)
        self._weight.widget.setValue(task.weight)
        self._refA.widget.setCurrentIndex(
            self._refA.widget.findData(task.frameA, RawDataRole)
        )
        self._refB.widget.setCurrentIndex(
            self._refB.widget.findData(task.frameB, RawDataRole)
        )
        self._rel_type.widget.setCurrentIndex(
            self._rel_type.widget.findData(task.relType, RawDataRole)
        )
        self._soft_type.widget.setCurrentIndex(
            self._soft_type.widget.findData(task.softnessType, RawDataRole)
        )

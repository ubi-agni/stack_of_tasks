import typing

from PyQt5 import QtWidgets

from stack_of_tasks.tasks import *
from stack_of_tasks.ui.hierarchy_tab.task_details import TaskDetails
from stack_of_tasks.ui.models import RawDataRole
from stack_of_tasks.ui.widgets import ButtonDialog

tasks = [PositionTask, ParallelTask, DistanceTask, OrientationTask, PlaneTask, ConeTask]


# TODO Sanity checks


class NewTask(ButtonDialog):
    def __init__(self, ref_model, parent: typing.Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)

        self.ref_model = ref_model
        self.data = TaskDetails()
        self.data.setModel(self.ref_model)

        self._cls = self.data.add_combo_row("Class", row=0)
        for t in tasks:
            self._cls.widget.addItem(t.name)

        self._vl.insertWidget(0, self.data)

        self._cls.widget.currentIndexChanged.connect(self.task_index_changed)
        self.task_index_changed(0)

    def task_index_changed(self, index: int):
        self.data._name.widget.setPlaceholderText(f"{tasks[index].name} task name")

    def generate_task(self):
        cls = tasks[self._cls.widget.currentIndex()]

        frameA = self.data._refA.widget.currentData(RawDataRole)
        frameB = self.data._refB.widget.currentData(RawDataRole)
        relType = self.data._rel_type.widget.currentData(RawDataRole)
        softType = self.data._soft_type.widget.currentData(RawDataRole)
        weight = self.data._weight.widget.value()

        return self.data._name.widget.text(), cls(frameA, frameB, softType, relType, weight)

from __future__ import annotations

from typing import Optional

from PyQt5 import QtWidgets
from stack_of_tasks_ui.generated.TaskHierachy import Ui_TaskHierarchy
from stack_of_tasks_ui.model import RawDataRole

from stack_of_tasks.tasks.Task import Task

from .new_task import NewTask


class HierarchyTab(QtWidgets.QWidget, Ui_TaskHierarchy):
    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)

        self.av_refs = None

        self.treeView.setDragDropMode(self.treeView.InternalMove)
        self.treeView.setDragEnabled(True)
        self.treeView.setAcceptDrops(True)
        self.treeView.setDropIndicatorShown(True)

        self.addTask.clicked.connect(self.add_task_action)

        self.task_details.hide()

    def set_task_hierarchy_model(self, model):
        self.treeView.setModel(model)
        self.treeView.selectionModel().selectionChanged.connect(self.tasks_selected)

    def set_ref_model(self, ref_model):
        self.av_refs = ref_model
        self.task_details.setModel(ref_model)

    def add_task_action(self):
        if (t := NewTask(self.av_refs)).exec() == NewTask.Accepted:
            name, taskInstance = t.generate_task()
            self.model.add_task(taskInstance, name)

    def tasks_selected(self):
        if len(sel := self.treeView.selectedIndexes()) > 0 and isinstance(
            item := sel[0].data(RawDataRole), Task
        ):
            self.task_details.set_trait_object(item)
            self.NoneSelected.hide()
            self.task_details.show()

        else:
            self.task_details.hide()
            self.NoneSelected.show()
            self.task_details.trait_form_layout.clear_widget()

from __future__ import annotations

from typing import Optional

from PyQt5 import QtWidgets

from stack_of_tasks.tasks.Task import RelativeTask
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy, TaskItem
from stack_of_tasks.ui.generated.TaskHierachy import Ui_TaskHierarchy
from stack_of_tasks.ui.model.task_hierarchy import TaskHierarchyModel

from .new_task import NewTask


class HierarchyTab(QtWidgets.QWidget, Ui_TaskHierarchy):
    def __init__(self, hierarchy: Optional[TaskHierarchy] = None) -> None:
        super().__init__()
        self.setupUi(self)

        self.av_refs = None
        self._selected_task: RelativeTask = None

        self.model = TaskHierarchyModel(hierarchy)

        self.treeView.setDragDropMode(self.treeView.InternalMove)
        self.treeView.setDragEnabled(True)
        self.treeView.setAcceptDrops(True)
        self.treeView.setDropIndicatorShown(True)
        self.treeView.setModel(self.model)

        self.addTask.clicked.connect(self.add_task_action)

        self.treeView.selectionModel().selectionChanged.connect(self.tasks_selected)

        self.task_details.hide()

    def set_ref_model(self, ref_model):
        self.av_refs = ref_model
        self.task_details.setModel(ref_model)

    def add_task_action(self):
        if (t := NewTask(self.av_refs)).exec() == NewTask.Accepted:
            name, taskInstance = t.generate_task()
            self.model.add_task(taskInstance, name)

    def tasks_selected(self):
        selection = self.treeView.selectedIndexes()

        if len(selection) > 0:
            if isinstance((item := selection[0].internalPointer()), TaskItem):
                self.NoneSelected.hide()
                self.task_details.show()

                self._selected_task: RelativeTask = item.task
                name = item.additional_data["name"]
                self.task_details.set_values_from_task(self._selected_task, name)
                return

        self.NoneSelected.show()
        self.task_details.hide()

from __future__ import annotations

import qtawesome as qta
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QToolBar

from stack_of_tasks.tasks.Task import Task
from stack_of_tasks.ui import RawDataRole
from stack_of_tasks.ui.generated.TaskHierachy import Ui_TaskHierarchy
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.utils.dependency_injection import DependencyInjection
from stack_of_tasks.ui.widgets.button_dialog import NewInstanceDialog


class HierarchyTab(QtWidgets.QWidget, Ui_TaskHierarchy):
    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)

        self.tool_bar = QToolBar()

        self.verticalLayout.insertWidget(0, self.tool_bar)

        self.add_action = self.tool_bar.addAction(qta.icon("fa.plus"), "add")
        self.remove_action = self.tool_bar.addAction(qta.icon("fa.trash-o"), "remove")
        self.remove_action.setDisabled(True)

        self.add_action.triggered.connect(self.add_task)
        self.remove_action.triggered.connect(self.remove_element)

        model = ModelMapping.get_mapping(Task)
        self.treeView.setModel(model)

        self.treeView.selectionModel().selectionChanged.connect(self.tasks_selected)

    def add_task(self):
        dialog = NewInstanceDialog(ModelMapping.get_mapping(ClassKey(Task)))

        if dialog.exec() == NewInstanceDialog.Accepted:
            cls = dialog.cls_selection.current_object
            args = dialog.traits.get_arguments()
            task = DependencyInjection.create_instance(cls, args)

            index = self.treeView.selectionModel().currentIndex()

            self.treeView.model().add_task(task, index)

    def remove_element(self):
        selected = self.treeView.selectionModel().currentIndex()
        self.treeView.model().remove_index(selected)

    def tasks_selected(self):
        if len(sel := self.treeView.selectedIndexes()) > 0:
            # task selected
            # obj = sel[0].data(RawDataRole)
            self.remove_action.setDisabled(False)

        else:
            self.remove_action.setDisabled(True)

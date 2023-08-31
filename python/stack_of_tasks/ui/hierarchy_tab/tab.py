from __future__ import annotations

import qtawesome as qta
from PyQt5.QtWidgets import QToolBar, QVBoxLayout, QWidget

from stack_of_tasks.tasks.Task import Task
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.property_tree.prop_tree import SOT_View
from stack_of_tasks.ui.utils.dependency_injection import DependencyInjection
from stack_of_tasks.ui.widgets.button_dialog import NewInstanceDialog


class HierarchyTab(QWidget):
    def __init__(self) -> None:
        super().__init__()

        self.verticalLayout = QVBoxLayout(self)
        self.setLayout(self.verticalLayout)

        self.tool_bar = QToolBar()

        self.verticalLayout.addWidget(self.tool_bar)

        self.treeView = SOT_View()
        self.verticalLayout.addWidget(self.treeView)

        self.add_action = self.tool_bar.addAction(qta.icon("fa.plus"), "add")
        self.remove_action = self.tool_bar.addAction(qta.icon("fa.trash-o"), "remove")
        self.remove_action.setDisabled(True)

        self.add_action.triggered.connect(self.add_task)
        self.remove_action.triggered.connect(self.remove_element)

        model = ModelMapping.get_mapping(Task)
        self.treeView.setModel(model)

        self.treeView.selectionModel().selectionChanged.connect(self.tasks_selected)

    def add_task(self):
        dialog = NewInstanceDialog(ModelMapping.get_mapping(ClassKey(Task)), self)

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
        if len(self.treeView.selectedIndexes()) > 0:
            self.remove_action.setDisabled(False)
        else:
            self.remove_action.setDisabled(True)

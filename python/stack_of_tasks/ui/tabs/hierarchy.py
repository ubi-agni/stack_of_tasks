from __future__ import annotations

from stack_of_tasks.tasks import Task
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.property_tree.prop_tree import SOT_View
from stack_of_tasks.ui.utils.dependency_injection import DependencyInjection
from stack_of_tasks.ui.widgets.button_dialog import NewInstanceDialog

from .base import Base


class HierarchyTabWidget(Base):
    def __init__(self) -> None:
        super().__init__()

        self.treeView = SOT_View()
        self.verticalLayout.addWidget(self.treeView)

        model = ModelMapping.get_mapping(Task)
        self.treeView.setModel(model)

        self.treeView.selectionModel().selectionChanged.connect(self.tasks_selected)

    def add_action_callback(self):
        dialog = NewInstanceDialog(ModelMapping.get_mapping(ClassKey(Task)), self)

        if dialog.exec() == NewInstanceDialog.Accepted:
            cls = dialog.cls_selection.current_object
            args = dialog.traits.get_arguments()
            task = DependencyInjection.create_instance(cls, args)

            self.treeView.add_tasks([task])

    def remove_action_callback(self):
        self.treeView.remove_selected()

    def tasks_selected(self):
        have_selection = len(self.treeView.selectedIndexes()) > 0
        self.remove_action.setEnabled(have_selection)

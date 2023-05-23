from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from stack_of_tasks.tasks.Task import Task
from stack_of_tasks.ui.generated.TaskHierachy import Ui_TaskHierarchy
from stack_of_tasks.ui.model import RawDataRole
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.new_instance_dialog import NewInstanceDialog


class HierarchyTab(QtWidgets.QWidget, Ui_TaskHierarchy):
    new_task_signal = QtCore.pyqtSignal([type, dict])

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
        if (
            t := NewInstanceDialog(ModelMapping.get_mapping(ClassKey(Task)))
        ).exec() == NewInstanceDialog.Accepted:
            cls = t.cls_selection.current_object
            args = t.traits.get_arguments()

            self.new_task_signal.emit(cls, args)

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

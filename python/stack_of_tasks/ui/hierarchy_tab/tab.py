from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from stack_of_tasks.tasks.Task import Task
from stack_of_tasks.ui.button_dialog import NewInstanceDialog
from stack_of_tasks.ui.generated.TaskHierachy import Ui_TaskHierarchy
from stack_of_tasks.ui.model import RawDataRole
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping


class HierarchyTab(QtWidgets.QWidget, Ui_TaskHierarchy):
    new_task_signal = QtCore.pyqtSignal([type, dict])

    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)

        self.treeView.setDragDropMode(self.treeView.InternalMove)
        self.treeView.setDragEnabled(True)
        self.treeView.setAcceptDrops(True)
        self.treeView.setDropIndicatorShown(True)

        self.addTask.clicked.connect(self.add_task_action)

        model = ModelMapping.get_mapping(Task)
        self.treeView.setModel(model)
        self.treeView.selectionModel().selectionChanged.connect(self.tasks_selected)

    def add_task_action(self):
        if (
            t := NewInstanceDialog(ModelMapping.get_mapping(ClassKey(Task)))
        ).exec() == NewInstanceDialog.Accepted:
            cls = t.cls_selection.current_object
            args = t.traits.get_arguments()

            self.new_task_signal.emit(cls, args)

    def tasks_selected(self):
        if len(sel := self.treeView.selectedIndexes()) > 0:
            obj = sel[0].data(RawDataRole)

            if not isinstance(obj, Task):
                obj = None
        else:
            obj = None

        self.edit_task.set_trait_object(obj)

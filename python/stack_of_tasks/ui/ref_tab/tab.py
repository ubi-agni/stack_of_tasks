from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from stack_of_tasks.ref_frame import RefFrame
from stack_of_tasks.ui.button_dialog import NewInstanceDialog
from stack_of_tasks.ui.generated.Refs import Ui_Refs
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.traits_mapping.custom_widgets.object_dropbown import (
    ObjectModel,
    RawDataRole,
)


class Ref_Tab(QtWidgets.QWidget, Ui_Refs):
    new_ref_signal = QtCore.pyqtSignal([type, dict])

    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)

        self.new_ref.setDefaultAction(self.actionadd_ref)
        self.actionadd_ref.triggered.connect(self.add_task_action)
        self.actionadd_ref.triggered.connect(lambda x: print("add ref"))
        self.ref_model: ObjectModel
        self.ref_details.hide()

    def set_model(self, model):
        self.ref_model = model
        self.ref_view.setModel(self.ref_model)
        self.ref_view.selectionModel().selectionChanged.connect(self.ref_selected)

    def add_task_action(self):
        if (
            t := NewInstanceDialog(ModelMapping.get_mapping(ClassKey(RefFrame)))
        ).exec() == NewInstanceDialog.Accepted:
            cls = t.cls_selection.current_object
            args = t.traits.get_arguments()

            self.new_ref_signal.emit(cls, args)

    def ref_selected(self):
        if len(sel := self.ref_view.selectedIndexes()) > 0:
            self.ref_details.set_trait_object(sel[0].data(RawDataRole))
            self.NoneSelected.hide()
            self.ref_details.show()

        else:
            self.ref_details.hide()
            self.NoneSelected.show()
            self.ref_details.trait_form_layout.clear_widget()

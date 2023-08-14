from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from stack_of_tasks.ref_frame import RefFrame
from stack_of_tasks.ui import RawDataRole
from stack_of_tasks.ui.generated.Refs import Ui_Refs
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.widgets.button_dialog import NewInstanceDialog


class Ref_Tab(QtWidgets.QWidget, Ui_Refs):
    new_ref_signal = QtCore.pyqtSignal([type, dict])

    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)

        self.addRef.clicked.connect(self.add_ref_action)
        self.actionadd_ref.triggered.connect(self.add_ref_action)
        self.actionadd_ref.triggered.connect(lambda x: print("add ref"))

        model = ModelMapping.get_mapping(RefFrame)

        self.ref_view.setModel(model)
        self.ref_view.selectionModel().selectionChanged.connect(self.ref_selected)

    def add_ref_action(self):
        if (
            t := NewInstanceDialog(ModelMapping.get_mapping(ClassKey(RefFrame)))
        ).exec() == NewInstanceDialog.Accepted:
            cls = t.cls_selection.current_object
            args = t.traits.get_arguments()
            self.new_ref_signal.emit(cls, args)

    def ref_selected(self):
        if len(sel := self.ref_view.selectedIndexes()) > 0:
            obj = sel[0].data(RawDataRole)
            self.edit_ref.set_trait_object(obj)
        else:
            obj = None

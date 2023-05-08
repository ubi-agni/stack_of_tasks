from __future__ import annotations

from PyQt5 import QtCore, QtWidgets
from stack_of_tasks_ui.generated.Refs import Ui_Refs
from stack_of_tasks_ui.model.av_ref import AvailableRefModel, RawDataRole
from stack_of_tasks_ui.ref_tab.new_ref import AddRef

from stack_of_tasks.ref_frame.offset import Offset


class Ref_Tab(QtWidgets.QWidget, Ui_Refs):
    new_marker_signal = QtCore.pyqtSignal([type, tuple])

    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)

        self.av_refs: AvailableRefModel = None
        self.selected_ref = None

        self.new_ref.setDefaultAction(self.actionadd_ref)
        self.actionadd_ref.triggered.connect(self.add_ref_triggered)

        self.ref_details._name.widget.textChanged.connect(self._name_changed)
        self.ref_details._root.widget.currentIndexChanged.connect(self._root_changed)
        self.ref_details.hide()

    def set_model(self, model):
        self.av_refs = model
        self.ref_view.setModel(model)
        self.ref_view.selectionModel().selectionChanged.connect(self.ref_selected)
        self.ref_details.set_model(self.av_refs)

    def add_ref_triggered(self, checked: bool):
        new_ref_dialog = AddRef(self.av_refs)

        if new_ref_dialog.exec() == AddRef.Accepted:
            ref, name = new_ref_dialog.generate_ref()
            self.av_refs.add_ref(ref, name)

    def _name_changed(self):
        print()
        if self.selected_ref is not None:
            self.av_refs.update_name(self.selected_ref, self.ref_details._name.widget.text())

    def _root_changed(self):
        if self.selected_ref is not None and isinstance(self.selected_ref, Offset):
            self.selected_ref.frame = self.ref_details._root.widget.currentData(RawDataRole)

    def ref_selected(self):
        # self.selected_ref.reset()
        selection = self.ref_view.selectedIndexes()

        if len(selection) == 1:
            self.NoneSelected.hide()
            self.ref_details.show()
            self.selected_ref = selection[0].data(RawDataRole)
            self.ref_details.set_values_from_ref(
                selection[0].data(RawDataRole), selection[0].data()
            )
            return

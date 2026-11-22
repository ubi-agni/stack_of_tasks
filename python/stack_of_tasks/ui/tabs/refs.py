from __future__ import annotations

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QHBoxLayout, QListView

from stack_of_tasks.ref_frame import Offset, RefFrame
from stack_of_tasks.ui import RawDataRole
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.widgets.button_dialog import NewInstanceDialog
from stack_of_tasks.ui.widgets.has_trait_widgets import EditorGroupBox

from .base import Base


class RefFramesTabWidget(Base):
    new_ref_signal = pyqtSignal([type, dict])

    def __init__(self, marker_server) -> None:
        super().__init__()

        self.marker_server = marker_server
        self._active = False

        self.horizontalLayout = QHBoxLayout()
        self.ref_view = QListView(self)
        self.horizontalLayout.addWidget(self.ref_view)

        self.edit_ref = EditorGroupBox(self)
        self.edit_ref.setTitle("properties:")
        self.horizontalLayout.addWidget(self.edit_ref)

        model = ModelMapping.get_mapping(RefFrame)
        self.ref_view.setModel(model)
        self.ref_view.selectionModel().selectionChanged.connect(self.ref_selected)

        self.verticalLayout.addLayout(self.horizontalLayout)

    def add_action_callback(self):
        if (
            t := NewInstanceDialog(ModelMapping.get_mapping(ClassKey(RefFrame)), self)
        ).exec() == NewInstanceDialog.Accepted:
            cls = t.cls_selection.current_object
            args = t.traits.get_arguments()
            self.new_ref_signal.emit(cls, args)

    def ref_selected(self):
        sel = self.ref_view.selectedIndexes()
        obj = sel[0].data(RawDataRole) if sel else None
        self.edit_ref.set_trait_object(obj)
        self.offset_control(obj if self._active else None)

    def set_active(self, active):
        self._active = active
        sel = self.ref_view.selectedIndexes()
        self.offset_control(sel[0].data(RawDataRole) if active and sel else None)

    def offset_control(self, obj):
        if isinstance(obj, Offset):
            self.marker_server.show_offset_marker(obj, "T")
        else:
            self.marker_server.hide_offset_marker()

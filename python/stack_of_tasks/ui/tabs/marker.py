from __future__ import annotations

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QHBoxLayout, QListView

from stack_of_tasks.marker import IAMarker
from stack_of_tasks.ui import RawDataRole
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.widgets.button_dialog import NewInstanceDialog
from stack_of_tasks.ui.widgets.has_trait_widgets import EditorGroupBox

from .base import Base


class MarkerTabWidget(Base):
    new_marker_signal = pyqtSignal([type, dict])

    def __init__(self) -> None:
        super().__init__()

        self.horizontalLayout = QHBoxLayout()
        self.marker_view = QListView()
        self.horizontalLayout.addWidget(self.marker_view)

        self.edit_marker = EditorGroupBox()
        self.edit_marker.setTitle("properties:")
        self.horizontalLayout.addWidget(self.edit_marker)

        model = ModelMapping.get_mapping(IAMarker)
        self.marker_view.setModel(model)
        self.marker_view.selectionModel().selectionChanged.connect(self.marker_selected)

        self.verticalLayout.addLayout(self.horizontalLayout)

    def add_action_callback(self):
        if (
            t := NewInstanceDialog(ModelMapping.get_mapping(ClassKey(IAMarker)), self)
        ).exec() == NewInstanceDialog.Accepted:
            cls = t.cls_selection.current_object
            args = t.traits.get_arguments()

            self.new_marker_signal.emit(cls, args)

    def marker_selected(self):
        if len(sel := self.marker_view.selectedIndexes()) > 0:
            obj = sel[0].data(RawDataRole)
        else:
            obj = None

        self.edit_marker.set_trait_object(obj)

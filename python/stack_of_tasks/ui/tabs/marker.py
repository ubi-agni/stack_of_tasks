from __future__ import annotations

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QHBoxLayout, QListView

from stack_of_tasks.marker import IAMarker
from stack_of_tasks.ui import RawDataRole
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.widgets.button_dialog import NewInstanceDialog
from stack_of_tasks.ui.widgets.has_trait_widgets import EditorGroupBox

from stack_of_tasks.ref_frame import MarkerFrame

from .base import Base


class MarkerTabWidget(Base):
    new_marker_signal = pyqtSignal([type, dict])
    remove_marker_signal = pyqtSignal(IAMarker)

    def __init__(self) -> None:
        super().__init__()

        self.horizontalLayout = QHBoxLayout()
        self.marker_view = QListView()
        self.horizontalLayout.addWidget(self.marker_view)

        self.edit_marker = EditorGroupBox()
        self.edit_marker.setTitle("properties:")
        self.horizontalLayout.addWidget(self.edit_marker)

        self.marker_view.setSelectionBehavior(QListView.SelectRows)
        self.marker_view.setSelectionMode(QListView.ExtendedSelection)

        model = ModelMapping.get_mapping(IAMarker)
        self.marker_view.setModel(model)
        self.marker_view.selectionModel().selectionChanged.connect(self.marker_selected)

        self.verticalLayout.addLayout(self.horizontalLayout)

    def add_action_callback(self):
        t = NewInstanceDialog(ModelMapping.get_mapping(ClassKey(IAMarker)), self)
        if t.exec() == NewInstanceDialog.Accepted:
            cls = t.cls_selection.current_object
            args = t.traits.get_arguments()
            self.new_marker_signal.emit(cls, args)

    def remove_action_callback(self):
        # delete all selected markers
        for index in self.marker_view.selectedIndexes():
            obj = index.data(RawDataRole)
            self.remove_marker_signal.emit(obj)

    def marker_selected(self):
        if len(sel := self.marker_view.selectedIndexes()) > 0:
            obj = sel[0].data(RawDataRole)
            #Check if the selected object is allowed to be removed.
            #If not, disable the button and show the error label
            for ref in self.ref_objects:
                if isinstance(ref, MarkerFrame):
                    if (ref.marker == obj):
                        self.edit_marker.set_trait_object(obj)

                        self.remove_action.setEnabled(False)
                        self.remove_action.setToolTip("Marker is in use by MarkerFrame " + ref.name)
                        return
        else:
            obj = None
        self.edit_marker.set_trait_object(obj)

        self.remove_action.setEnabled(obj != None)
        self.remove_action.setToolTip("remove")

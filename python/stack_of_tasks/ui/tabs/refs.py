from __future__ import annotations

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QHBoxLayout, QListView

from stack_of_tasks.ref_frame import RefFrame
from stack_of_tasks.ui import RawDataRole
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.widgets.button_dialog import NewInstanceDialog
from stack_of_tasks.ui.widgets.has_trait_widgets import EditorGroupBox
from stack_of_tasks.ref_frame import Offset

from .base import Base

from stack_of_tasks.marker import IAMarker
from stack_of_tasks.marker.trait_marker import FullMovementMarker


class RefFramesTabWidget(Base):
    new_ref_signal = pyqtSignal([type, dict])

    marker_server = None

    def __init__(self, marker_server) -> None:
        super().__init__()

        self.marker_server = marker_server

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
            #Create a new marker for each Offset, which it then stores
            if cls == Offset:
                self.marker_server.add_marker(marker := FullMovementMarker(name=args["name"]+"_offset_control__", frame_id=IAMarker._default_frame_id))
                args["marker"] = marker
                marker.set_visibility(False)
            self.new_ref_signal.emit(cls, args)

    def ref_selected(self):
        if len(sel := self.ref_view.selectedIndexes()) > 0:
            obj = sel[0].data(RawDataRole)
            self.edit_ref.set_trait_object(obj)

            if isinstance(obj, Offset):
                #If an Offset is selected hide all markers...
                for marker in self.marker_server.marker.values():
                    marker.set_visibility(False)
                #except the one that controls the selected Offset
                obj.marker.set_visibility(True)
                obj.marker.transform = obj.T
            else:
                #Else show all normal markers, then hide the offset markers...
                for marker in self.marker_server.marker.values():
                    marker.set_visibility(True)
                #by looping over all references in this view...
                model = self.ref_view.model()
                for row in range(model.rowCount()):
                    objAtRow = model.data(model.index(row, 0), RawDataRole)
                    #and if the reference is an Offset, hiding its marker
                    if isinstance(objAtRow, Offset):
                        objAtRow.marker.set_visibility(False)
        else:
            obj = None

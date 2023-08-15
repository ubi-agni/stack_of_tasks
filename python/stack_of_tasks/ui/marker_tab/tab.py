from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from stack_of_tasks.marker import IAMarker
from stack_of_tasks.ui import RawDataRole
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.widgets.button_dialog import NewInstanceDialog
from stack_of_tasks.ui.widgets.has_trait_widgets import EditorGroupBox


class MarkerTab(QtWidgets.QWidget):
    new_marker_signal = QtCore.pyqtSignal([type, dict])

    def __init__(self) -> None:
        super().__init__()

        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.marker_view = QtWidgets.QListView()
        self.horizontalLayout.addWidget(self.marker_view)

        self.editor_layout = QtWidgets.QVBoxLayout()
        self.edit_marker = EditorGroupBox()
        self.editor_layout.addWidget(self.edit_marker)
        self.add_marker = QtWidgets.QPushButton("add marker")
        self.editor_layout.addWidget(self.add_marker)
        self.horizontalLayout.addLayout(self.editor_layout)

        self.add_marker.clicked.connect(self.add_button_action)
        self.setLayout(self.horizontalLayout)

        model = ModelMapping.get_mapping(IAMarker)
        self.marker_view.setModel(model)
        self.marker_view.selectionModel().selectionChanged.connect(self.marker_selected)

    def add_button_action(self):
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

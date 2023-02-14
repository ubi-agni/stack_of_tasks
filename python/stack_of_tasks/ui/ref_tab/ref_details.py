from __future__ import annotations

from stack_of_tasks.ref_frame import JointFrame, Offset, RefFrame, World
from stack_of_tasks.ui.data_input import DataInput
from stack_of_tasks.ui.models import NumpyTableModel, RawDataRole, RefFramesModel
from stack_of_tasks.ui.widgets import RefComboBox


class Ref_Details(DataInput):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self._refs_model: RefFramesModel = None
        self._matrix_model = NumpyTableModel()

        self._name = self.add_string_row("Name")
        self._root = self.add_combo_row("Parent", combo=RefComboBox())

        self._transform = self.add_matrix_row("T", model=self._matrix_model)

    def set_model(self, model: RefFramesModel):
        self._refs_model = model
        self._root.widget.setModel(self._refs_model)

    def _set_selection(self, obj):
        for i in range(self._refs_model.rowCount()):
            ind = self._refs_model.index(i)
            print(ind.data())

    def set_values_from_ref(self, ref: RefFrame | Offset, name: str):
        self._name.widget.setText(name)

        if isinstance(ref, RefFrame):
            self._name.widget.setReadOnly(isinstance(ref, (World, JointFrame)))
            self._matrix_model.setMatrix(ref.T.view())
            self._transform.label.setText("T")
            self._transform.widget.setEnabled(False)
            self._root.setVisible(False)
        else:
            self._name.widget.setReadOnly(False)
            ref.callback.append(lambda _: self._matrix_model.valuesChanged())
            self._matrix_model.setMatrix(ref.offset.view())
            self._transform.label.setText("Offset")
            self._transform.widget.setEnabled(True)

            self._root.widget.setCurrentIndex(
                self._root.widget.findData(ref.frame, RawDataRole)
            )
            self._root.setVisible(True)

from __future__ import annotations

from enum import Enum

import typing

from PyQt5.QtWidgets import QWidget

from stack_of_tasks.ref_frame.offset import HasJacobian, Offset, OffsetWithJacobian
from stack_of_tasks.ui.models import EnumModel, RawDataRole
from stack_of_tasks.ui.ref_tab.ref_details import Ref_Details
from stack_of_tasks.ui.widgets import ButtonDialog


class RefTypes(Enum):
    """Docstring for MyEnum."""

    Offset = "Offset"


class AddRef(ButtonDialog):

    av_refs_mode = EnumModel(RefTypes)

    def __init__(self, refs, parent: typing.Optional[QWidget] = None) -> None:
        super().__init__(parent)

        self.ref_model = refs
        self.data = Ref_Details()
        self.data.set_model(refs)
        self.data._name.widget.setPlaceholderText("Ref name")
        self._vl.insertWidget(0, self.data)

    def generate_ref(self):
        root = self.data._root.widget.currentData(RawDataRole)
        cls = OffsetWithJacobian if isinstance(root, HasJacobian) else Offset
        return cls(root, self.data._matrix_model._matrix), self.data._name.widget.text()

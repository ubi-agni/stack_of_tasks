from __future__ import annotations

from enum import Enum

import typing
from typing import List

import numpy as np
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFormLayout,
    QListView,
    QPushButton,
    QStyledItemDelegate,
    QTreeView,
    QWidget,
)

from stack_of_tasks.ref_frame.frames import JointFrame, RefFrame, World
from stack_of_tasks.ref_frame.offset import HasJacobian, Offset, OffsetWithJacobian
from stack_of_tasks.ui.ButtonDialog import ButtonDialog
from stack_of_tasks.ui.generated.NewRefDialog import Ui_NewRefDialog
from stack_of_tasks.ui.model import RawDataRole
from stack_of_tasks.ui.model.enum_model import EnumModel, RawDataRole

from .ref_details import Ref_Details


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
        root = self.data.rootRefComboBox.currentData(RawDataRole)
        cls = OffsetWithJacobian if isinstance(root, HasJacobian) else Offset
        return cls(root, self.data.v._data), self.data.nameLineEdit.text()

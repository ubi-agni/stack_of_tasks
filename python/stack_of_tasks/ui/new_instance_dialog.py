import typing

from PyQt5 import QtWidgets

from stack_of_tasks.ui.ButtonDialog import ButtonDialog
from stack_of_tasks.ui.traits_mapping import NewInstanceWidget
from stack_of_tasks.ui.traits_mapping.custom_widgets.object_dropbown import (
    ObjectDropdown,
    ObjectModel,
)


class NewInstanceDialog(ButtonDialog):
    def __init__(
        self, model: ObjectModel, parent: typing.Optional[QtWidgets.QWidget] = None
    ) -> None:
        super().__init__(parent)

        self.cls_selection = ObjectDropdown()
        self.cls_selection.current_object_changed.connect(self.cls_selection_changed)

        fl = QtWidgets.QFormLayout()
        fl.addRow("Class:", self.cls_selection)
        self._vl.insertLayout(0, fl)
        self.traits = NewInstanceWidget()
        self._vl.insertWidget(1, self.traits)

        self.cls_selection.setModel(model)

    def cls_selection_changed(self, cls):
        self.traits.cls = cls
        self.traits._setup_widgets()

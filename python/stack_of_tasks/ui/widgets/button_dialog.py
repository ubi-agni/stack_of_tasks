import typing

from PyQt5 import QtWidgets

from stack_of_tasks.ui.model.object_model import ObjectModel


class ButtonDialog(QtWidgets.QDialog):
    def __init__(self, parent: None) -> None:
        super().__init__(parent)

        self._vl = QtWidgets.QVBoxLayout()
        self.setLayout(self._vl)
        self.buttonBox = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )

        self._vl.addWidget(self.buttonBox)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)


class NewInstanceDialog(ButtonDialog):
    def __init__(
        self, model: ObjectModel, parent: typing.Optional[QtWidgets.QWidget] = None
    ) -> None:
        super().__init__(parent)

        self.cls_selection = ObjectDropdown()

        fl = QtWidgets.QFormLayout()
        fl.addRow("Class:", self.cls_selection)
        self._vl.insertLayout(0, fl)
        self.traits = NewInstanceWidget()
        self._vl.insertWidget(1, self.traits)

        self.cls_selection.current_object_changed.connect(self.cls_selection_changed)
        self.cls_selection.setModel(model)

    def cls_selection_changed(self, cls):
        self.traits.cls = cls
        self.traits._setup_widgets()


from stack_of_tasks.ui.widgets.has_trait_widgets import NewInstanceWidget
from stack_of_tasks.ui.widgets.object_dropbown import ObjectDropdown

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

        #If defining the Label here, the text needs to be changed from somewhere else,
        #since the trait that may be invalid is not known here
        self.testWidget = QtWidgets.QLabel("Input invalid")
        self.testWidget.setStyleSheet("color: red;")
        self.testWidget.setHidden(True)
        self._vl.addWidget(self.testWidget)

        self._vl.addWidget(self.buttonBox)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)


class NewInstanceDialog(ButtonDialog):
    def __init__(self, model: ObjectModel, parent: typing.Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)

        self.cls_selection = ObjectDropdown()

        fl = QtWidgets.QFormLayout()
        fl.addRow("Class:", self.cls_selection)
        self._vl.insertLayout(0, fl)
        self.traits = NewInstanceWidget()
        self._vl.insertWidget(1, self.traits)

        self.cls_selection.current_object_changed.connect(self.cls_selection_changed)
        self.cls_selection.setModel(model)

        #This enables/disables the OK button depending on if the input is valid.
        #However, there is no reason given to the user -> add toggleable QLabel somewhere?
        self.traits.input_change.connect(self.buttonBox.button(QtWidgets.QDialogButtonBox.Ok).setEnabled)
        self.traits.input_change.connect(self.testWidget.setHidden)
        #self.traits.testing.connect(self.testWidget.text)

    def cls_selection_changed(self, cls):
        self.traits.cls = cls
        self.traits._setup_widgets()


from stack_of_tasks.ui.widgets.has_trait_widgets import NewInstanceWidget
from stack_of_tasks.ui.widgets.object_dropbown import ObjectDropdown

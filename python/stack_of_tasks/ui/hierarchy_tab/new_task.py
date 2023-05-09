import typing

from PyQt5 import QtWidgets

from stack_of_tasks.ui.ButtonDialog import ButtonDialog
from stack_of_tasks.ui.hierarchy_tab.task_details import TaskDetails
from stack_of_tasks.ui.model import RawDataRole
from stack_of_tasks.ui.traits_mapping import HasTraitPropertyMixin
from stack_of_tasks.ui.traits_mapping.custom_widgets.object_dropbown import ObjectDropdown

# TODO Sanity checks


class NewTask(ButtonDialog):
    def __init__(self, parent: typing.Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)

        self.cls_selection = ObjectDropdown()
        # trait to get args

    def cls_selection_changed(self):
        pass

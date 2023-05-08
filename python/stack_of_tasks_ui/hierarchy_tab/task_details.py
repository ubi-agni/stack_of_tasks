from PyQt5.QtWidgets import QWidget
from stack_of_tasks_ui.data_input import DataInput
from stack_of_tasks_ui.model.enum_model import EnumModel, RawDataRole
from stack_of_tasks_ui.traits_mapping import HasTraitGroupBox, HasTraitPropertyMixin

from stack_of_tasks.tasks.Task import RelativeTask, TaskSoftnessType


class TaskDetails(HasTraitPropertyMixin):
    pass

from PyQt5.QtWidgets import QTabWidget
from stack_of_tasks_ui.generated.CentralTabWidgit import Ui_TabWidget


class TabWidget(QTabWidget, Ui_TabWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setupUi(self)

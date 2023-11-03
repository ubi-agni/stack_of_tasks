from __future__ import annotations

import qtawesome as qta
from PyQt5.QtWidgets import QToolBar, QVBoxLayout, QWidget


class Base(QWidget):
    def __init__(self) -> None:
        super().__init__()

        self.verticalLayout = QVBoxLayout(self)
        self.setLayout(self.verticalLayout)

        self.tool_bar = QToolBar()

        self.verticalLayout.addWidget(self.tool_bar)

        self.add_action = self.tool_bar.addAction(qta.icon("fa.plus"), "add")
        self.remove_action = self.tool_bar.addAction(qta.icon("fa.trash-o"), "remove")
        self.remove_action.setDisabled(True)

        self.add_action.triggered.connect(self.add_action_callback)
        self.remove_action.triggered.connect(self.remove_action_callback)

    def add_action_callback(self):
        pass

    def remove_action_callback(self):
        pass

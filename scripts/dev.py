import random
import sys

from PyQt5 import QtGui
from PyQt5.QtCore import QModelIndex, QStringListModel, Qt, pyqtSlot
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QCompleter,
    QLineEdit,
    QVBoxLayout,
    QWidget,
)

from stack_of_tasks.ref_frame import JointFrame, World
from stack_of_tasks.robot_model import RobotModel, RobotState
from stack_of_tasks.ui.models import RefFramesModel
from stack_of_tasks.ui.widgets import RefComboBox


class Window(QWidget):
    def __init__(self):
        super().__init__()
        robot_state = RobotState(RobotModel())
        self.model = RefFramesModel(robot_state)
        self.model.add_ref(World(), "ROOT")
        self.model.add_ref(JointFrame(robot_state, "panda_hand_tcp"), "eef")

        l = QVBoxLayout()
        self.setLayout(l)
        l.addWidget(QLineEdit())

        for _ in range(3):
            combo = RefComboBox()
            l.addWidget(combo)
            combo.setModel(self.model)

        self.show()


app = QApplication(sys.argv)
window = Window()
sys.exit(app.exec())

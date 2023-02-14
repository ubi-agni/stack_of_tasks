import random
import sys

from PyQt5 import QtGui
from PyQt5.QtCore import QModelIndex, QStringListModel, Qt, pyqtSlot
from PyQt5.QtWidgets import QApplication, QComboBox, QLineEdit, QVBoxLayout, QWidget

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

        self.combo = RefComboBox()

        lineedit = QLineEdit()
        lineedit.setCompleter(self.combo.completer())

        lineedit.textChanged.connect(self.update_combo)

        l = QVBoxLayout()
        self.setLayout(l)
        l.addWidget(lineedit)
        l.addWidget(self.combo)
        self.show()

        self.combo.setModel(self.model)
        self.combo.setCurrentIndex(self.combo.findData("ROOT", Qt.DisplayRole))

    @pyqtSlot(str)
    def update_combo(self, text: str):
        idx = self.combo.findData(text, Qt.DisplayRole)
        self.combo.setCurrentIndex(idx)


app = QApplication(sys.argv)
window = Window()
sys.exit(app.exec())

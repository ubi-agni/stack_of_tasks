import random
import sys

from PyQt5 import QtGui
from PyQt5.QtCore import QModelIndex, QStringListModel, Qt, pyqtSlot
from PyQt5.QtWidgets import QApplication, QComboBox, QDialog, QLineEdit, QVBoxLayout

from stack_of_tasks.ref_frame import JointFrame, World
from stack_of_tasks.robot_model import RobotModel, RobotState
from stack_of_tasks.ui.models import RefFramesModel


class ComboBox(QComboBox):
    other = ["Germany", "USA", "France"]

    def __init__(self, model: QStringListModel, editable=True, parent=None) -> None:
        super().__init__(parent)
        self.setEditable(editable)
        self.setInsertPolicy(QComboBox.NoInsert)
        if model is not None:
            self.setModel(model)

    def focusInEvent(self, e: QtGui.QFocusEvent) -> None:
        self.update_completer_model()  # update completer model when entering
        super().focusInEvent(e)

    @pyqtSlot()
    def update_completer_model(self):
        completer = self.completer()
        if completer is not None:
            root = QModelIndex()
            m = self.model()
            m = QStringListModel(
                [m.index(r, 0, root).data(Qt.DisplayRole) for r in range(m.rowCount(root))]
                + self.other
                + [f"#{100*(i+1) + random.randint(0, 99)}" for i in range(3)]
            )
            completer.setModel(m)


class Window(QDialog):
    def __init__(self):
        super().__init__()
        robot_state = RobotState(RobotModel())
        self.model = RefFramesModel(robot_state)
        self.model.add_ref(World(), "ROOT")
        self.model.add_ref(JointFrame(robot_state, "panda_hand_tcp"), "eef")

        # QStringListModel works!
        # self.model = QStringListModel(["Berlin", "Tokyo", "New York"])
        self.combo = ComboBox(self.model)

        lineedit = QLineEdit()
        lineedit.setCompleter(self.combo.completer())

        lineedit.textChanged.connect(self.update_combo)

        l = QVBoxLayout()
        self.setLayout(l)
        l.addWidget(lineedit)
        l.addWidget(self.combo)
        self.show()

    @pyqtSlot(str)
    def update_combo(self, text: str):
        idx = self.combo.findData(text, Qt.DisplayRole)
        self.combo.setCurrentIndex(idx)


app = QApplication(sys.argv)
window = Window()
sys.exit(app.exec())

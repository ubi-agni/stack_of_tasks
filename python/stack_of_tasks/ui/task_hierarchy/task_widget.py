import qtawesome as qta
from click import argument
from PyQt5.QtCore import QMimeData, QSize, Qt, pyqtSignal
from PyQt5.QtGui import (
    QDrag,
    QDragEnterEvent,
    QDragMoveEvent,
    QDropEvent,
    QIcon,
    QMouseEvent,
    QPixmap,
)
from PyQt5.QtWidgets import QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout
from yaml import emit

from stack_of_tasks.tasks.Task import Task as TaskData
from stack_of_tasks.ui.Collapse import CollapseWithDelete


class ArgumentDropdown(QComboBox):

    argument_map_changed = pyqtSignal(str, str)

    def __init__(
        self,
        key,
        initial_value: str = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.key = key
        self.emit = True

        if initial_value:
            self.addItem(initial_value)

        def selection_changed(text):
            if self.emit:
                self.argument_map_changed.emit(self.key, text)

        self.currentTextChanged.connect(selection_changed)

    def list_changed(self, targets: list):
        self.emit = False
        currentItem = self.currentText()
        fa5_icon = qta.icon("fa.exclamation")

        self.clear()

        if currentItem in targets:
            i = targets.index(currentItem)
            self.addItems(targets)
            self.setCurrentIndex(i)
        else:
            self.addItem(currentItem)
            self.addItems(targets)
            self.setCurrentIndex(0)
            self.setItemIcon(0, fa5_icon)
        self.emit = True


class TaskWidget(CollapseWithDelete):
    target_list_changed_signal = pyqtSignal(list)

    def __init__(self, task, parent=None, flags=None) -> None:
        self.task: TaskData = task

        super().__init__(parent, self.task.name)
        self._contentLayout = QFormLayout()
        self._contentLayout.setContentsMargins(20, 5, 20, 5)
        self.content.setLayout(self._contentLayout)

        self._scale_slider = QDoubleSpinBox()
        self._scale_slider.setMinimum(0.01)
        self._scale_slider.setValue(self.task._scale)
        self._scale_slider.setSingleStep(0.01)
        self._scale_slider.setMaximum(10)
        self._scale_slider.setButtonSymbols(QDoubleSpinBox.PlusMinus)
        self._contentLayout.addRow("scale", self._scale_slider)

        self._is_hard = QCheckBox()
        self._contentLayout.addRow("is hard", self._is_hard)

        def set_arg(arg, val):
            self.task.argmap[arg] = val
            print(self.task.argmap)

        for k in list(self.task.argmap.keys()):
            ad = ArgumentDropdown(key=k, initial_value=str(k))

            self.target_list_changed_signal.connect(ad.list_changed)
            ad.argument_map_changed.connect(set_arg)

            self._contentLayout.addRow(str(k), ad)

    def _set_arg(self, arg, val):
        self.task.argmap[arg] = val

    def mouseMoveEvent(self, e: QMouseEvent) -> None:

        if e.buttons() == Qt.LeftButton:
            drag = QDrag(self)
            drag.setMimeData(QMimeData())
            drag.setPixmap(self.grab())

            result: Qt.DropAction = drag.exec_(Qt.MoveAction)

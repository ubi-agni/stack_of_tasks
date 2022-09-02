from cmath import e
from operator import index
from typing import Iterable, List, Optional, Union

from PyQt5.QtCore import QMimeData, Qt, pyqtSignal
from PyQt5.QtGui import (
    QDrag,
    QDragEnterEvent,
    QDragMoveEvent,
    QDropEvent,
    QMouseEvent,
    QPixmap,
)
from PyQt5.QtWidgets import QApplication, QLabel, QListWidgetItem, QPushButton, QWidget
from sympy import re

from stack_of_tasks.ui.generated.TaskHierarchy import Ui_TaskHierarchy
from stack_of_tasks.ui.utils import TASK_MIME_FOTMAT_NAME, TASKS, mime_data_from_name

from .level_widget import LevelWidget
from .task_widget import TaskWidget


class TaskHierarchy(QWidget, Ui_TaskHierarchy):

    new_task = pyqtSignal(str, int)
    move_level_signal = pyqtSignal(int, int)
    move_task_signal = pyqtSignal(object, int, int)
    delete_task_signal = pyqtSignal(int, object)
    delete_level_signal = pyqtSignal(int)

    target_data_changed = pyqtSignal(list)

    def __init__(self, parent=None) -> None:
        super().__init__(parent=parent)
        self.setupUi(self)
        self.hierarchy.layout().setAlignment(Qt.AlignTop)

        self.hierarchy.setAcceptDrops(True)
        self.tasks.mimeData = self._generate_task_mime

        self.hierarchy.dragEnterEvent = self._hierarchy_drag_enter
        self.hierarchy.dropEvent = self._hierarchy_drop_event

        self.available_targets = set()

        for t in TASKS.values():
            self.tasks.addItem(t["display_name"])

    # Drag from av. tasks

    def _generate_task_mime(self, items: Iterable[QListWidgetItem]):
        data = QMimeData()
        if len(items) == 1:
            byte_data = mime_data_from_name(items[0].text())
            data.setData(TASK_MIME_FOTMAT_NAME, byte_data)
        else:
            pass
        return data

    def _hierarchy_drag_enter(self, e: QDragEnterEvent):
        e.accept()

    def _find_item_at(self, pos):
        for n in range(self.hierarchy.layout().count()):
            w: QWidget = self.hierarchy.layout().itemAt(n).widget()
            r = w.rect()
            r.moveTo(w.pos())
            if r.contains(pos):
                return w

    def _hierarchy_drop_event(self, e: QDropEvent):
        pos = e.pos()
        n = -1
        if w := self._find_item_at(pos):
            n = self.hierarchy.layout().indexOf(w)

        if e.mimeData().hasFormat(TASK_MIME_FOTMAT_NAME):
            task_name = e.mimeData().data(TASK_MIME_FOTMAT_NAME).data().decode()
            self.new_task.emit(task_name, n)

        elif (widget := e.source()) and isinstance(widget, LevelWidget):
            # move level
            self.move_level_signal.emit(widget.level, n)

            i1 = self.hierarchy.layout().itemAt(widget.level).widget()
            self.hierarchy.layout().insertWidget(n, i1)
            for i in range(self.hierarchy.layout().count()):
                self.hierarchy.layout().itemAt(i).widget().set_level(i)

        elif (widget := e.source()) and isinstance(widget, TaskWidget):
            # move task
            task = widget.task
            parent = widget.parent().parent().level
            self.move_task_signal.emit(task, parent, n)

            self.hierarchy.layout().itemAt(n).widget().add_task_widget(widget)

        else:
            return e.ignore()

        e.accept()

    # hierarchy manipulation

    def delete_task(self, level_index, task):
        level_widget = self.hierarchy.layout().itemAt(index).widget()

    def delete_level(self, index):
        level_widget = self.hierarchy.layout().itemAt(index).widget()

        self.hierarchy.layout().removeWidget(level_widget)
        level_widget.setParent(None)
        level_widget.destroy()

        for i in range(index, self.hierarchy.layout().count()):
            self.hierarchy.layout().itemAt(i).widget().set_level(i)

    def add_task_at_level(self, task, level, av_targets):
        levels = self.hierarchy.layout().count()

        if level < levels:
            s: LevelWidget = self.hierarchy.layout().itemAt(level).widget()
        else:
            s: LevelWidget = LevelWidget(level)
            s.delete_task_signal.connect(self.delete_task_signal.emit)
            s.delete_self.connect(lambda: self.delete_level(s.level))
            s.delete_self.connect(lambda: self.delete_level_signal.emit(s.level))
            self.hierarchy.layout().addWidget(s)

        tw = TaskWidget(task)
        self.target_data_changed.connect(tw.target_list_changed_signal.emit)
        tw.target_list_changed_signal.emit(av_targets)
        s.add_task_widget(tw)

    # set available targets
    def set_available_targets(self, list_of_targets):

        self.target_data_changed.emit(list_of_targets)

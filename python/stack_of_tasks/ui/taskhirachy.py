

from typing import Iterable

from PyQt5 import QtCore, QtGui, QtWidgets
from stack_of_tasks.task import Task as TaskData
from stack_of_tasks.task import __available_task_classes__

from . import TASK_MIME_FOTMAT_NAME
from .Collapse import CollapseWithDelete
from .generated.TaskHirachy import Ui_TaskHirachy


class TaskWidget(CollapseWithDelete):
    def __init__(self, task, parent=None, flags=None) -> None:
        self.task:TaskData = task
        super().__init__(parent, self.task.name)

        self._contentLayout = QtWidgets.QFormLayout()
        self._contentLayout.setContentsMargins(20, 5,20,5)
        self.content.setLayout(self._contentLayout)
        
        for k in self.task.args:
            self._contentLayout.addRow(str(k), QtWidgets.QComboBox())


class LevelWidget(CollapseWithDelete):
    def __init__(self, level:int, parent=None ) -> None:
        self.level = level
        super().__init__(parent, f"Level {self.level}")
        self._contentLayout = QtWidgets.QVBoxLayout()
        self._contentLayout.setContentsMargins(20, 5,0,5)
        self.content.setLayout(self._contentLayout)
        self.setAcceptDrops(True)


    def delete_task(self, index):
        w = self._contentLayout.itemAt(index)
        self._contentLayout.removeItem(w)
        w.widget().deleteLater()

    def add_task(self, task):
        t = TaskWidget(task)

        def _del_task(widget):
            index = self._contentLayout.indexOf(widget)
            self.delete_task(index)
        
        t.delete_self.connect(lambda:_del_task(t))
        self._contentLayout.addWidget(t)
        self.updateGeometry()


    def dragEnterEvent(self, e:QtGui.QDragEnterEvent):
        e.accept()

    def dragMoveEvent(self, e:QtGui.QDragMoveEvent):
        return super().dragMoveEvent(e)
    
    def dropEvent(self, e:QtGui.QDropEvent):
        if e.mimeData().hasFormat(TASK_MIME_FOTMAT_NAME):
            task_name = e.mimeData().data(TASK_MIME_FOTMAT_NAME).data().decode()

            self.add_task(task_name)
            e.accept()
        else:
            e.ignore()

import sys


class TaskHirachy(QtWidgets.QWidget, Ui_TaskHirachy):
    def __init__(self, parent=None) -> None:
        super().__init__(parent=parent)
        self.setupUi(self)
        self.hirachy.layout().setAlignment(QtCore.Qt.AlignTop)
        
        self.hirachy.setAcceptDrops(True)
        self.tasks.mimeData = self._generate_task_mime

        self.hirachy.dragEnterEvent = self._hirachy_drag_enter
        self.hirachy.dragMoveEvent = self._hirachy_drag_move
        self.hirachy.dropEvent = self._hirachy_drop_event

        for t in __available_task_classes__:
            self.tasks.addItem(t.name)

    def _generate_task_mime(self, items:Iterable[QtWidgets.QListWidgetItem]):
        data = QtCore.QMimeData()
        if len(items) == 1:
            data.setData(TASK_MIME_FOTMAT_NAME, items[0].text().encode())
        else:
            pass
        return data


    def _hirachy_drag_enter(self, e:QtGui.QDragEnterEvent):
        e.accept()

    def _hirachy_drag_move(self, e:QtGui.QDragMoveEvent):
        return super().dragMoveEvent(e)
    
    def _hirachy_drop_event(self, e:QtGui.QDropEvent):
        if e.mimeData().hasFormat(TASK_MIME_FOTMAT_NAME):
            task_name = e.mimeData().data(TASK_MIME_FOTMAT_NAME).data().decode()
            
            for t in __available_task_classes__:
                if t.name == task_name:
                    task = t
                    break

            level = self.hirachy.layout().count()
            self.add_task_at_level(task, level)
            e.accept()
        else:
            e.ignore()

    def delete_level(self, level_widget):
        index = self.hirachy.layout().indexOf(level_widget)
        self.hirachy.layout().removeWidget(level_widget)
        level_widget.setParent(None)
        level_widget.destroy()

        for i in range(index, self.hirachy.layout().count()):
            self.hirachy.layout().itemAt(i).setText()

    def add_task_at_level(self, task, level):
        levels = self.hirachy.layout().count()
        if level < levels:
            s:LevelWidget = self.hirachy.layout().itemAt(level).widget()
        else:
            s:LevelWidget = LevelWidget(level)
            s.delete_self.connect(lambda: self.delete_level(s))
            self.hirachy.layout().addWidget(s)
        
        s.add_task(task)


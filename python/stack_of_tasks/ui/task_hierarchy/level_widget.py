from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtCore import Qt, QMimeData, pyqtSignal
from PyQt5.QtGui import QDrag, QPixmap, QDragEnterEvent, QDragMoveEvent, QDropEvent, QMouseEvent

from stack_of_tasks.ui.utils  import TASK_MIME_FOTMAT_NAME, TASKS, mime_data_from_name

from stack_of_tasks.ui.Collapse import CollapseWithDelete

from .task_widget import TaskWidget

class LevelWidget(CollapseWithDelete):

    delete_task_signal = pyqtSignal(int, object)

    def __init__(self, level:int, parent=None ) -> None:
        self.level = level

        super().__init__(parent, f"Level {self.level}")
        self._contentLayout = QVBoxLayout()
        self._contentLayout.setContentsMargins(20, 5,0,5)
        self.content.setLayout(self._contentLayout)

    def set_level(self, level):
        self.level = level
        self.toggle_button.setText(f"Level {self.level}")

    def delete_task(self, index):
        w = self._contentLayout.itemAt(index)
        self._contentLayout.removeItem(w)
        w.widget().deleteLater()

    def add_task_widget(self, task_widget):
        def _del_task(widget):
            index = self._contentLayout.indexOf(widget)
            self.delete_task_signal.emit(self.level, widget.task)
            self.delete_task(index)
        
        task_widget.delete_self.connect(lambda:_del_task(task_widget))
        self._contentLayout.addWidget(task_widget)
        self.updateGeometry()


    def mouseMoveEvent(self, e: QMouseEvent) -> None:
        if e.buttons() == Qt.LeftButton:
            
            drag = QDrag(self)
            drag.setMimeData(QMimeData())
            drag.setPixmap(self.grab())

            result:Qt.DropAction = drag.exec_(Qt.MoveAction)


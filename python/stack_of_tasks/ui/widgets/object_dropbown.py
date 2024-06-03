from __future__ import annotations

from logging import Logger

from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import QVariant, pyqtProperty, pyqtSignal
from PyQt5.QtGui import QPainter, QPaintEvent
from PyQt5.QtWidgets import QComboBox, QStyle, QStyleOptionButton, QStyleOptionComboBox

from stack_of_tasks.ui import RawDataRole
from stack_of_tasks.ui.model.object_model import ObjectModel

logger = Logger(__name__)


class ObjectDropdown(QComboBox):
    current_object_changed = pyqtSignal(object)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.currentIndexChanged.connect(
            lambda: self.current_object_changed.emit(self.current_object)
        )

        self._button_state = QStyle.State_Raised

    def get_current_object(self):
        return self.currentData(RawDataRole)

    def set_current_object(self, val):
        if val is not None:
            val_index = self.model().find(val)
            if val_index is None:
                return
            else:
                self.setCurrentIndex(val_index)

    current_object = pyqtProperty(
        QVariant,
        get_current_object,
        set_current_object,
        notify=current_object_changed,
        user=True,
    )


class AddableObjectDropdown(ObjectDropdown):
    add_button_clicked = pyqtSignal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self._button_state = QStyle.State_Raised
        self.cls_model = None

        self.add_button_clicked.connect(self.create_new_element)

    def create_new_element(self):
        d = NewInstanceDialog(self.cls_model, self)
        if d.exec() == NewInstanceDialog.Accepted:
            args = d.traits.get_arguments()
            new_data = d.traits.cls(**args)
            self.model().append(new_data)
            self.set_current_object(new_data)

    def sizeHint(self) -> QtCore.QSize:
        margin = self.style().pixelMetric(QStyle.PM_LayoutLeftMargin)
        s = super().sizeHint()
        s = s.grownBy(QtCore.QMargins(0, 0, s.height() + margin, 0))
        return s

    def mouseReleaseEvent(self, e: QtGui.QMouseEvent) -> None:
        s = self.size()
        r = QtCore.QRect(s.width() - s.height(), 0, s.height(), s.height())

        if r.contains(e.pos(), True):
            self._button_state = QStyle.State_Raised
            self.repaint()
            self.add_button_clicked.emit()
        else:
            super().mouseReleaseEvent(e)

    def mousePressEvent(self, e: QtGui.QMouseEvent) -> None:
        s = self.size()
        r = QtCore.QRect(s.width() - s.height(), 0, s.height(), s.height())

        if r.contains(e.pos(), True):
            self._button_state = QStyle.State_Sunken
            self.repaint()
        else:
            super().mousePressEvent(e)

    def leaveEvent(self, a0: QtCore.QEvent) -> None:
        self._button_state = QStyle.State_Raised
        return super().leaveEvent(a0)

    def paintEvent(self, e: QPaintEvent) -> None:
        style = self.style()

        cb_option = QStyleOptionComboBox()

        self.initStyleOption(cb_option)

        margin = self.style().pixelMetric(QStyle.PM_LayoutLeftMargin)
        cb_option.rect.adjust(0, 0, -cb_option.rect.height() - margin, 0)

        painter = QPainter(self)
        self.initPainter(painter)
        style.drawComplexControl(style.CC_ComboBox, cb_option, painter, self)
        style.drawControl(style.CE_ComboBoxLabel, cb_option, painter, self)

        button_option = QStyleOptionButton()
        button_option.initFrom(self)

        button_option.icon = QtGui.QIcon.fromTheme("add")
        button_option.iconSize = cb_option.iconSize

        button_option.state |= self._button_state
        button_option.rect = QtCore.QRect(
            cb_option.rect.right() + margin,
            0,
            cb_option.rect.height(),
            cb_option.rect.height(),
        )

        style.drawControl(style.CE_PushButton, button_option, painter)


from .button_dialog import NewInstanceDialog

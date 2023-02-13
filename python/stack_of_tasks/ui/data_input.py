from typing import Dict, Generic, Literal, TypeVar, Union, overload

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QComboBox, QDoubleSpinBox, QFormLayout, QLabel, QLineEdit, QWidget

from .widgets import *

PropType = TypeVar("PropType", bound=QWidget)


class Prop(Generic[PropType]):
    def __init__(self, labelWidgit: QLabel, propWidgit: PropType) -> None:
        self.label: QLabel = labelWidgit
        self.widget: PropType = propWidgit

    def setVisible(self, val):
        self.label.setVisible(val)
        self.widget.setVisible(val)


class DataInput(Generic[PropType], QWidget):
    def __init__(self, parent=None, flags=0) -> None:
        super().__init__(parent)

        self._fl = QFormLayout()
        self.setLayout(self._fl)
        self._props: Dict[str, Prop] = {}

    property_changed = pyqtSignal(str, object)

    def _add_prop(self, l: str, w: QWidget, row: int):
        lw = QLabel(l)
        if not w.objectName():
            w.setObjectName(l)
        prop = Prop(lw, w)
        self._props[l] = prop
        if row is not None:
            self._fl.insertRow(row, lw, w)
        else:
            self._fl.addRow(lw, w)
        return prop

    def add_float_row(
        self, label, min=0.0, max=1.0, step=0.1, default=1, row: int = None
    ) -> Prop[QDoubleSpinBox]:
        w = QDoubleSpinBox()
        w.setMinimum(min)
        w.setMaximum(max)
        w.setSingleStep(step)
        w.setValue(default)
        return self._add_prop(label, w, row)

    def add_string_row(self, label, placeholder=None, row: int = None) -> Prop[QLineEdit]:
        w = QLineEdit()
        if placeholder is not None:
            w.setPlaceholderText(placeholder)
        return self._add_prop(label, w, row)

    def add_combo_row(
        self, label, model=None, row: int = None, combo=None
    ) -> Prop[QComboBox]:
        if not isinstance(combo, QComboBox):
            combo = QComboBox()
        if model is not None:
            combo.setModel(model)
        return self._add_prop(label, combo, row)

    def add_matrix_row(self, label, row: int = None, model=None) -> Prop[MatrixView]:
        w = MatrixView(model=model)
        return self._add_prop(label, w, row)

    def addPropertyCallback(self, name, cb):
        def propFilter(l_name, l_value):
            if name == l_name:
                cb(l_value)

        self.property_changed.connect(propFilter)

    def _fire_property(self, name, func=None):
        def f(*args, **kwargs):
            if func is not None:
                self.property_changed.emit(name, func(*args, **kwargs))
            else:
                self.property_changed.emit(name, *args)

        return f

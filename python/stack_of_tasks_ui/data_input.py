from typing import Dict, Generic, Literal, TypeVar, Union, overload

from PyQt5.QtCore import QModelIndex, pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QLabel,
    QLineEdit,
    QTreeView,
    QWidget,
)

from stack_of_tasks.ui.matrix_view import MatrixView
from stack_of_tasks.ui.model import RawDataRole

PropType = TypeVar("PropType", bound=QWidget)


class TreeComboBox(QComboBox):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self._view = QTreeView()
        self._view.setHeaderHidden(True)
        self._view.collapsed.connect(self._view_height_changed)
        self._view.expanded.connect(self._view_height_changed)

        self.setView(self._view)

    def showPopup(self):
        super().showPopup()
        self._view_height_changed()

    def _view_height_changed(self):
        sh = self._view.viewportSizeHint()
        rh = self._view.sizeHintForRow(0) + 2 * self._view.autoScrollMargin()
        sh.setHeight(max(sh.height(), rh))
        self._view.window().resize(sh)

    def findIndexOfObject(self, obj, role=RawDataRole):

        for i in range(self.model().rowCount(QModelIndex())):
            ii = self.model().index(i, 0, QModelIndex())

            for j in range(self.model().rowCount(ii)):
                ij = self.model().index(j, 0, ii)

                if ij.data(role) is obj:
                    return ij
        return QModelIndex()

    def setTreeIndex(self, index: QModelIndex):
        if index.isValid():
            self.setRootModelIndex(index.parent())
            self.setModelColumn(index.column())
            self.setCurrentIndex(index.row())
            self.setRootModelIndex(QModelIndex())
            self.view().setCurrentIndex(index)
        else:
            self.setRootModelIndex(QModelIndex())
            self.setCurrentIndex(-1)


class Prop(Generic[PropType]):
    def __init__(self, labelWidgit: QLabel, propWidgit: PropType) -> None:
        self.label: QLabel = labelWidgit
        self.widget: PropType = propWidgit

    def setVisible(self, val):
        self.label.setVisible(val)
        self.widget.setVisible(val)


class DataInput(Generic[PropType], QWidget):
    def __init__(
        self,
        parent=None,
        flags=0,
    ) -> None:
        super().__init__(parent)

        self._fl = QFormLayout()
        self.setLayout(self._fl)
        self._props: Dict[str, Prop] = {}

    property_changed = pyqtSignal(str, object)

    def _add_prop(self, l: str, w, row: int):
        lw = QLabel(l)
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

    @overload
    def add_combo_row(
        self, label, model=None, row: int = None, treeView: Literal[False] = False
    ) -> Prop[QComboBox]:
        pass

    @overload
    def add_combo_row(
        self, label, model=None, row: int = None, treeView: Literal[True] = True
    ) -> Prop[TreeComboBox]:
        pass

    def add_combo_row(
        self, label, model=None, row: int = None, treeView=False
    ) -> Union[Prop[QComboBox], Prop[TreeComboBox]]:
        w = TreeComboBox() if treeView else QComboBox()
        if model is not None:
            w.setModel(model)
        return self._add_prop(label, w, row)

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

from enum import Enum
from random import uniform

from PyQt5.QtCore import QLocale, QStringListModel
from PyQt5.QtGui import QValidator
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from qt_binder.raw_widgets import DoubleSpinBox
from qt_binder.widgets import Composite, EnumDropDown, LineEdit, List, on_trait_change
from traits.api import HasTraits, Instance, Range


class MyEnum(Enum):
    """Docstring for MyEnum."""

    FIRST_ENUM = "some_value"
    SECOND_ENUM = "some_other_value"


class ToyTask(HasTraits):
    weight = Range(low=0.0, value=1.0, exclude_low=True)

    softness = List([(e, e.name) for e in MyEnum])


class RangeTraitSpinBox(DoubleSpinBox):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.singleStep = 0.01


class DevMainWindow(Composite):
    qclass = QMainWindow

    spin_box = Instance(RangeTraitSpinBox, args=())
    drop_down = Instance(EnumDropDown, args=())
    selection = Instance(LineEdit, args=())

    data = Instance(ToyTask)

    def construct(self, *args, **kwds):
        print("construct")
        self.spin_box.construct()
        self.drop_down.construct()
        self.selection.construct()

        super(DevMainWindow, self).construct()
        self.qobj.setProperty("binder_class", "RangeSlider")

        button1 = QPushButton("randomize trait")
        button2 = QPushButton("randomize widget")
        button1.clicked.connect(self._r_data)
        button2.clicked.connect(self._r_ui)

        w = QWidget()
        l = QVBoxLayout()
        w.setLayout(l)
        l.addWidget(self.spin_box.qobj)
        l.addWidget(self.drop_down.qobj)
        l.addWidget(self.selection.qobj)
        l.addWidget(button1)
        l.addWidget(button2)

        self.centralWidget = w
        self.show()

    def configure(self):
        super().configure()
        self.selection.readOnly = True

    @on_trait_change("data")
    def _data_changed(self):
        self.data.sync_trait("softness", self.drop_down, "values")
        self.data.sync_trait("weight", self.spin_box, "value")

    @on_trait_change("drop_down.value")
    def _dropdown_selection_changed(self):
        self.selection.text = self.drop_down.value.value

    def _r_data(self):
        self.data.trait_set(weight=uniform(0.001, 100))

    def _r_ui(self):
        self.spin_box.setValue(uniform(0.001, 100))


if __name__ == "__main__":
    a = QApplication([])

    data = ToyTask()
    data.observe(print, "weight")
    x = DevMainWindow()
    x.construct()
    x.configure()
    x.data = data
    print(x.child_binders)

    a.exec()

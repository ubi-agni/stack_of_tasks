from random import uniform

import typing

from PyQt5.QtCore import QLocale
from PyQt5.QtGui import QValidator
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from traits.api import HasTraits, Range, TraitError
from traits.observation.events import TraitChangeEvent


class ToyTask(HasTraits):
    weight = Range(
        low=0.0,
        value=1.0,
        exclude_low=True,
        desc="Tasks weight on its level of hierarchy.",
    )


class RangeTraitSpinBox(QDoubleSpinBox):
    def __init__(self, object, trait, parent=None) -> None:
        super().__init__(parent)

        self.set_trait(object, trait)

        self.setSingleStep(0.01)
        self._step_enables = self.StepNone
        self._check_step_enabled()

    def set_trait(self, object: HasTraits, name: str):
        self._obj: HasTraits = object
        self._trait_name = name

        trait: Range = self._obj.trait(self._trait_name).handler

        if (l := trait._low) is not None:
            self.setMinimum(l)
        else:
            self.setMinimum(-10e10)

        if (h := trait._high) is not None:
            self.setMinimum(h)
        else:
            self.setMaximum(10e10)

        self.setValue(getattr(self._obj, self._trait_name))

        self._obj.observe(self._trait_changed_event, self._trait_name)
        self.valueChanged.connect(lambda val: setattr(self._obj, self._trait_name, val))

        self.exclude_high = trait._exclude_high
        self.exclude_low = trait._exclude_low

    def _trait_changed_event(self, evt: TraitChangeEvent):
        self.setValue(evt.new)

    def stepEnabled(self) -> QDoubleSpinBox.StepEnabled:
        return self._step_enables

    def _is_valid_value(self, value):
        try:
            self._obj.validate_trait(self._trait_name, value)
        except TraitError:
            return False
        else:
            return True

    def _check_step_enabled(self):
        self._step_enables = self.StepNone

        if self._is_valid_value(self.value() - self.singleStep()):
            self._step_enables |= self.StepDownEnabled
        if self._is_valid_value(self.value() + self.singleStep()):
            self._step_enables |= self.StepUpEnabled

    def stepBy(self, steps: int) -> None:
        if self._is_valid_value(v := self.value() + self.singleStep() * steps):
            self.setValue(v)
        self._check_step_enabled()

    def validate(self, input: str, pos: int) -> typing.Tuple[QValidator.State, str, int]:
        if len(input) > 0:
            val, converted = QLocale().toDouble(input)

            if converted and self._is_valid_value(val):
                return QValidator.Acceptable, input, pos

        return QValidator.Intermediate, input, pos


class MainWindow(QMainWindow):
    def __init__(self, task, parent=None) -> None:
        super().__init__(parent)

        self.task = task

        self.spin_box = RangeTraitSpinBox(task, "weight")
        button1 = QPushButton("randomize trait")
        button2 = QPushButton("randomize widget")
        button1.clicked.connect(self._r_trait)
        button2.clicked.connect(self._r_ui)

        l = QVBoxLayout()
        l.addWidget(self.spin_box)
        l.addWidget(button1)
        l.addWidget(button2)

        w = QWidget()
        w.setLayout(l)
        self.setCentralWidget(w)

        self.show()

    def _r_trait(self):
        self.task.trait_set(weight=uniform(0.001, 100))
        print(
            "Info - produces two change events, because the spinbox rounds the data on setting, producing a new value.\n"
        )

    def _r_ui(self):
        self.spin_box.setValue(uniform(0.001, 100))


if __name__ == "__main__":
    a = QApplication([])

    task = ToyTask()
    task.observe(print, "weight")

    x = MainWindow(task)

    a.exec()

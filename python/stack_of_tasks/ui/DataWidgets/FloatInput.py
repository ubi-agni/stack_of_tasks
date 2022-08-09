from PyQt5.QtCore import QSize, Qt, pyqtSignal
from PyQt5.QtWidgets import QDoubleSpinBox, QHBoxLayout, QSlider, QWidget


class FloatInput(QWidget):

    value_changed = pyqtSignal(float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.slider = QSlider(Qt.Horizontal)

        self.number = QDoubleSpinBox()

        self.number.valueChanged.connect(lambda x: self.slider.setValue(int(x * 100)))
        self.slider.valueChanged.connect(lambda x: self.number.setValue(x / 100.0))
        self.slider.rangeChanged.connect(
            lambda m, M: self.number.setRange(m / 100.0, M / 100.0)
        )
        self.value_changed = self.number.valueChanged

        self.value_changed.connect(lambda x: print(x))
        self.setRange(-10, 10)
        self.setValue(0)
        layout = QHBoxLayout()
        layout.addWidget(self.number)
        layout.addWidget(self.slider)
        self.setLayout(layout)

    def setRange(self, m, M):
        self.slider.setRange(m * 100, M * 100)

    def setValue(self, value):
        self.number.setValue(value)

    def value(self):
        return self.number.value()

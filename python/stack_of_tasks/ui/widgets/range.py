from __future__ import annotations

from enum import IntFlag

import PyQt5.QtWidgets as QTW
from PyQt5.QtWidgets import QAbstractSpinBox


class HasBound(IntFlag):
    NO_BOUND = 0x0
    HAS_MIN = 0x1
    HAS_MAX = 0x2


class ExcludeBound(IntFlag):
    EXCLUDE_NONE = 0x0
    EXCLUDE_MIN = 0x1
    EXCLUDE_MAX = 0x2


class Range(QTW.QDoubleSpinBox):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.exclude_bound = ExcludeBound.EXCLUDE_NONE
        self.has_bound = HasBound.HAS_MIN | HasBound.HAS_MAX

    def setMaximum(self, max: float | None) -> None:
        if max is not None:
            super().setMaximum(max)
        else:
            self.has_bound ^= HasBound.HAS_MAX

    def setMinimum(self, min: float | None) -> None:
        if min is not None:
            super().setMinimum(min)
        else:
            self.has_bound ^= HasBound.HAS_MIN

    def setExcludeLow(self, val):
        if val:
            self.exclude_bound |= ExcludeBound.EXCLUDE_MIN
        else:
            self.exclude_bound ^= ExcludeBound.EXCLUDE_MIN

    def setExcludeHigh(self, val):
        if val:
            self.exclude_bound |= ExcludeBound.EXCLUDE_MAX
        else:
            self.exclude_bound ^= ExcludeBound.EXCLUDE_MAX

    def stepEnabled(self) -> QAbstractSpinBox.StepEnabled:
        f = QAbstractSpinBox.StepNone

        if (
            (hb := HasBound.HAS_MIN in self.has_bound)
            and (
                (ex := ExcludeBound.EXCLUDE_MIN in self.exclude_bound)
                and self.value() - self.singleStep() > self.minimum()
            )
            or (not ex and self.value() - self.singleStep() >= self.minimum())
        ) or not hb:
            f |= QAbstractSpinBox.StepDownEnabled

        if (
            (hb := HasBound.HAS_MAX in self.has_bound)
            and (
                (ex := ExcludeBound.EXCLUDE_MAX in self.exclude_bound)
                and self.value() + self.singleStep() < self.maximum()
            )
            or (not ex and self.value() + self.singleStep() <= self.maximum())
        ) or not hb:
            f |= QAbstractSpinBox.StepUpEnabled
        return f

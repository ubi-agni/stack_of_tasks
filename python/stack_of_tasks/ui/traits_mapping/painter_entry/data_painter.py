from __future__ import annotations

from itertools import chain

from typing import Any

import numpy as np
from PyQt5.QtCore import QPointF, QRect, QSize, Qt
from PyQt5.QtGui import QPainter, QPen
from PyQt5.QtWidgets import QApplication, QStyle, QStyleOptionViewItem


class Delegate_Painter:
    @classmethod
    def size_hint(cls, option: QStyleOptionViewItem, data: Any):
        return QSize()

    has_Painter = False

    @classmethod
    def paint(cls, painter: QPainter, option: QStyleOptionViewItem, data: Any):
        painter.save()
        if option.state & QStyle.State_Selected:
            painter.fillRect(option.rect, option.palette.highlight())
        painter.restore()


class MatrixPainter(Delegate_Painter):
    has_Painter = True

    @classmethod
    def size_hint(cls, option: QStyleOptionViewItem, data: np.ndarray):
        if data is None:
            return super().size_hint(option, data)

        style = QApplication.style()
        m = style.pixelMetric(style.PM_ButtonMargin, option)
        f = style.pixelMetric(style.PM_DefaultFrameWidth, option)

        max_w = option.fontMetrics.averageCharWidth() * 10
        max_h = option.fontMetrics.height()

        if len(data.shape) == 1:
            data = data.reshape((1, -1))
        h = data.shape[0] * (max_h + f)
        w = data.shape[1] * (max_w + f)

        return QSize(w, h)

    @classmethod
    def paint(cls, painter: QPainter, option: QStyleOptionViewItem, data: np.ndarray) -> None:
        super().paint(painter, option, data)
        if data is None or data.size == 0:
            return

        painter.save()

        if len(data.shape) == 1:
            data = data.reshape((1, -1))
        shape = data.shape
        rect = option.rect
        content_size = cls.size_hint(option, data)
        rect.setSize(content_size)

        if (m := shape[0]) > 0:
            dm = int(rect.height() / m)
        else:
            dm = 0

        lns = list(
            chain.from_iterable(
                (
                    QPointF(rect.left(), rect.top() + dm * i),
                    QPointF(rect.right(), rect.top() + dm * i),
                )
                for i in range(1, m)
            )
        )

        n = shape[1]
        dn = int(rect.width() / n)

        lns.extend(
            chain.from_iterable(
                (
                    QPointF(rect.left() + dn * i, rect.top()),
                    QPointF(rect.left() + dn * i, rect.bottom()),
                )
                for i in range(1, n)
            )
        )

        if len(lns) > 0:
            painter.save()
            line_pen = QPen(option.palette.dark().color(), 0.2)
            painter.setPen(line_pen)
            painter.drawLines(*lns)

            painter.restore()

        text_role = (
            option.palette.HighlightedText
            if option.state & QStyle.State_Selected
            else option.palette.Text
        )

        for j in range(m):
            for i in range(n):
                QApplication.style().drawItemText(
                    painter,
                    QRect(rect.left() + i * dn, rect.top() + j * dm, dn, dm),
                    Qt.AlignCenter,
                    option.palette,
                    True,
                    f"{data[j, i]:.2e}",
                    text_role,
                )

        painter.restore()
        return True

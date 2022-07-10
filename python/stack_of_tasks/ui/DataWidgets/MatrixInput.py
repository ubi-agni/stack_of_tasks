from numpy import array, ndarray
from PyQt5.QtCore import QSize, Qt, pyqtSignal
from PyQt5.QtWidgets import QScrollArea, QSizePolicy, QTableWidget, QTableWidgetItem


class MatrixWidgetItem(QTableWidgetItem):

    FloatRole = Qt.UserRole + 1

    def __init__(self, default_value=0):
        super().__init__()

        self.setData(MatrixWidgetItem.FloatRole, default_value)
        self.setFlags(self.flags() & ~Qt.ItemIsSelectable)

    def setData(self, role: int, value) -> None:
        if role == MatrixWidgetItem.FloatRole:
            super().setData(MatrixWidgetItem.FloatRole, value)
            super().setData(Qt.DisplayRole, f"{value :.4f}")

        elif role == Qt.EditRole:
            try:
                value = float(value)
                self.setData(MatrixWidgetItem.FloatRole, value)

            except ValueError:
                self.setData(MatrixWidgetItem.FloatRole, None)
        else:
            super().setData(role, value)

    def data(self, role: int):
        if role == Qt.EditRole:
            v = self.data(MatrixWidgetItem.FloatRole)
            return str(v)
        return super().data(role)


class MatrixInput(QTableWidget):

    value_changed = pyqtSignal(ndarray)

    def __init__(self, rows=1, cols=1, colNames=[], rowNames=[], parent=None):
        super().__init__(parent=parent)
        self.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        self.setSizeAdjustPolicy(QScrollArea.AdjustToContents)

        rows = max(rows, len(rowNames))
        cols = max(cols, len(colNames))

        self.setRowCount(rows)
        self.setColumnCount(cols)

        if rowNames:
            self.setVerticalHeaderLabels(rowNames)

        if len(colNames) > 0:
            self.setHorizontalHeaderLabels(colNames)
        else:
            self.horizontalHeader().hide()

        if len(rowNames) > 0:
            self.setVerticalHeaderLabels(colNames)
        else:
            self.verticalHeader().hide()

        for c in range(self.columnCount()):
            for r in range(self.rowCount()):
                self.setItem(r, c, MatrixWidgetItem())

    def value(self):
        return self.toNumpy()

    def setValue(self, value: ndarray):
        rVal, cVal = value.shape
        rCur, cCur = self.rowCount(), self.columnCount()

        self.setRowCount(rVal)
        self.setColumnCount(cVal)

        for r in range(rCur, rVal):
            for c in range(self.columnCount()):
                self.setItem(r, c, MatrixWidgetItem())

        for r in range(self.rowCount()):
            for c in range(cCur, cVal):
                self.setItem(r, c, MatrixWidgetItem())

        for r in range(rVal):
            for c in range(cVal):
                self.item(r, c).setData(MatrixWidgetItem.FloatRole, value[r, c])

        self.resizeRowsToContents()
        self.resizeColumnsToContents()

    def toNumpy(self):
        return array(self.toList())

    def toList(self):
        l = []
        for r in range(self.rowCount()):
            sl = []
            for c in range(self.columnCount()):
                sl.append(self.item(r, c).data(MatrixWidgetItem.FloatRole))
            l.append(sl)
        return l

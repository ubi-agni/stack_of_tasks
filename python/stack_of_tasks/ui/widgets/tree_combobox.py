from PyQt5.QtCore import QModelIndex
from PyQt5.QtWidgets import QComboBox, QTreeView

from stack_of_tasks.ui.models import RawDataRole


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

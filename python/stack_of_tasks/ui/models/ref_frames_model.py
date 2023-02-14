import os
from pathlib import Path

import typing
from typing import List, Tuple, Union

import rospkg
from PyQt5.QtCore import QAbstractListModel, QModelIndex, QStringListModel, Qt
from PyQt5.QtGui import QIcon

from stack_of_tasks.ref_frame import JointFrame, Offset, RefFrame
from stack_of_tasks.robot_model import RobotState

from . import RawDataRole


class RefFramesModel(QAbstractListModel):
    def __init__(self, robot_state: RobotState, parent=None) -> None:
        super().__init__(parent)
        self._robot_state = robot_state
        self._refs: List[Tuple[str, Union[RefFrame, Offset]]] = []

        icon_path = Path(rospkg.RosPack().get_path("rviz")) / "icons" / "classes"
        self.target_icon = QIcon(str(icon_path / "Axes.png"))
        self.link_icon = QIcon(str(icon_path / "RobotLink.png"))

    def rowCount(self, parent: QModelIndex) -> int:
        return 0 if parent.isValid() else len(self._refs)

    def sibling(self, row: int, column: int, parent: QModelIndex) -> QModelIndex:
        if not parent.isValid() or column != 0 or row >= len(self._refs) or row < 0:
            return QModelIndex()
        return self.createIndex(row, 0)

    def flags(self, index: QModelIndex) -> Qt.ItemFlags:
        if not index.isValid():
            return super().flags(index)
        _, ref = self._refs[index.row()]
        extra_flags = Qt.ItemIsEditable if isinstance(ref, Offset) else 0
        return super().flags(index) | extra_flags

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole) -> typing.Any:
        if index.row() < 0 or index.row() >= len(self._refs):
            return

        name, ref = self._refs[index.row()]
        if role == Qt.DisplayRole or role == Qt.EditRole:
            return name
        elif role == RawDataRole:
            return ref
        elif role == Qt.DecorationRole:
            if isinstance(ref, JointFrame):
                return self.link_icon
            else:
                return self.target_icon

    def setData(self, index: QModelIndex, value: typing.Any, role: int) -> bool:
        if index.row() < 0 or index.row() >= len(self._refs):
            return False

        row = index.row()
        name, ref = self._refs[row]
        if role == Qt.EditRole:
            if name == value:
                return True
            self._refs[row] = (value, ref)
        elif role == RawDataRole:
            if ref is value:
                return True
            self._refs[row] = (name, value)
        self.dataChanged.emit(index, index)
        return True

    def update_name(self, ref, name):
        row = self._find(ref)
        self.setData(self.createIndex(row, 0), name, Qt.EditRole)

    def add_ref(self, ref, name):
        if self._find(ref) >= 0:
            return  # already have this ref
        first = len(self._refs)
        self.beginInsertRows(QModelIndex(), first, first + 1)
        self._refs.append((name, ref))
        self.endInsertRows()

    def ref(self, name: str) -> RefFrame:
        """Return reference with given name"""
        for n, ref in self._refs:
            if n == name:
                return ref

    def allRefs(self) -> QStringListModel:
        frames = set([name for name, _ in self._refs])
        frames.update(self._robot_state.robot_model.links.keys())
        return QStringListModel(sorted(frames))

    def _find(self, ref: RefFrame) -> int:
        for i, (_, r) in enumerate(self._refs):
            if r is ref:
                return i
        return -1

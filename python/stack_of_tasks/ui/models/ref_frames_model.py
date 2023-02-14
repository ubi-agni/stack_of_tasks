import os
from pathlib import Path

import typing
from typing import List, Tuple

import rospkg
from PyQt5.QtCore import QAbstractItemModel, QModelIndex, QStringListModel, Qt
from PyQt5.QtGui import QIcon

from stack_of_tasks.ref_frame.frames import JointFrame, RefFrame
from stack_of_tasks.robot_model import RobotState

from . import RawDataRole


class RefFramesModel(QAbstractItemModel):
    def __init__(self, robot_state: RobotState, parent=None) -> None:
        super().__init__(parent)
        self._robot_state = robot_state
        self._refs: List[Tuple[RefFrame]] = []

        icon_path = Path(rospkg.RosPack().get_path("rviz")) / "icons" / "classes"
        self.target_icon = QIcon(str(icon_path / "Axes.png"))
        self.link_icon = QIcon(str(icon_path / "RobotLink.png"))

    def rowCount(self, parent: QModelIndex) -> int:
        return 0 if parent.isValid() else len(self._refs)

    def columnCount(self, parent: QModelIndex) -> int:
        return 1

    def parent(self, child: QModelIndex) -> QModelIndex:
        return QModelIndex()

    def index(self, row: int, column: int, parent: QModelIndex) -> QModelIndex:
        return QModelIndex() if parent.isValid() else self.createIndex(row, column, None)

    def flags(self, index: QModelIndex) -> Qt.ItemFlags:
        flags = Qt.ItemIsEnabled | Qt.ItemIsSelectable
        return flags

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole) -> typing.Any:
        if not index.isValid():
            return

        name, ref = self._refs[index.row()]
        if role == Qt.DisplayRole:
            return name
        elif role == RawDataRole:
            return ref
        elif role == Qt.DecorationRole:
            if isinstance(ref, JointFrame):
                return self.link_icon
            else:
                return self.target_icon

    def setData(self, index: QModelIndex, value: typing.Any, role: int) -> bool:
        row = index.row()
        name, ref = self._refs[row]
        if role == Qt.EditRole:
            self._refs[row] = (value, ref)
        elif role == RawDataRole:
            self._refs[row] = (name, value)
        self.dataChanged.emit(index, index)

    def update_name(self, ref, name):
        row = self._find(ref)
        self.setData(self.createIndex(row, 0, None), name, Qt.EditRole)

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

import typing
from typing import Dict, List

from PyQt5.QtCore import QAbstractItemModel, QModelIndex, Qt

from stack_of_tasks.ref_frame.frames import JointFrame

from . import RawDataRole


class AvailableRefModel(QAbstractItemModel):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self._additional_data = []

        self._sections = []
        self._refs: Dict = {}

    def _data_to_section(self, data) -> str:
        if isinstance(data, JointFrame):
            return "Joint Frames"
        else:
            return "Other"

    def rowCount(self, parent=None) -> int:
        if (item := parent.internalPointer()) is None:
            # print("index", item, len(self._sections))
            return len(self._sections)
        elif isinstance(item, str):
            # print("index", item, len(self._refs[item]))
            return len(self._refs[item])
        return 0

    def columnCount(self, parent: QModelIndex = ...) -> int:
        return 1

    def parent(self, child: QModelIndex) -> QModelIndex:
        if (valid := child.isValid()) and (item := child.internalPointer()) is not None:

            if isinstance(item, dict):
                parent = None
                for k, v in self._refs.items():
                    if item in v:
                        parent = k
                        break

                if parent is not None:
                    return self.createIndex(self._sections.index(parent), 0, parent)

        return QModelIndex()

    def index(self, row: int, column: int, parent: QModelIndex = None) -> QModelIndex:
        p = None
        if parent is not None and parent.isValid():
            p = parent.internalPointer()

        if p is None and row < len(self._sections):
            return self.createIndex(row, 0, self._sections[row])

        elif isinstance(p, str):
            return self.createIndex(row, 0, self._refs[p][row])

        return QModelIndex()

    def flags(self, index: QModelIndex) -> Qt.ItemFlags:
        item = index.internalPointer()
        flags = Qt.ItemIsEnabled
        if isinstance(item, dict):
            flags |= Qt.ItemIsSelectable
        return flags

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole) -> typing.Any:
        item = index.internalPointer()
        if role == Qt.DisplayRole:
            if isinstance(item, str):
                return item
            else:
                return item["name"]

        elif role == RawDataRole:
            if isinstance(item, dict):
                return item["obj"]

    def update_name(self, ref, name):
        section = self._data_to_section(ref)

        for i, x in enumerate(self._refs[section]):
            if x["obj"] is ref:
                index = self.index(i, 0, self.index(self._sections.index(section), 0))
                x["name"] = name
                self.dataChanged.emit(index, index)

    def add_ref(self, ref, name):
        self.beginInsertRows(QModelIndex(), len(self._refs), len(self._refs))
        x = {"name": name, "obj": ref}
        section = self._data_to_section(ref)

        if section in self._sections:
            self._refs[section].append(x)
        else:
            self._sections.append(section)
            self._refs[section] = [x]

        self.endInsertRows()

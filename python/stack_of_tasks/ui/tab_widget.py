from PyQt5.QtWidgets import QTabWidget

from stack_of_tasks.ui.tabs import Hierarchy, Marker, Parameter, RefFrames, SolverSettings


class TabWidget(QTabWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.solver = SolverSettings()
        self.addTab(self.solver, "Solver")

        self.hierarchy = Hierarchy()
        self.addTab(self.hierarchy, "Hierarchy")

        self.refs = RefFrames()
        self.addTab(self.refs, "Refs")

        self.marker = Marker()
        self.addTab(self.marker, "Marker")

        self.parameter = Parameter()
        self.addTab(self.parameter, "Parameter")

        # TabWidget.setCurrentIndex(1)

#!/usr/bin/env python3

from random import random

import traits.api as ta
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication

from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.solver.OSQPSolver import OSQPSolver
from stack_of_tasks.tasks.Eq_Tasks import ParallelTask, PositionTask
from stack_of_tasks.tasks.Task import Task, TaskSoftnessType
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy
from stack_of_tasks.ui.mainwindow import Ui
from stack_of_tasks.ui.model.task_hierarchy import TaskHierarchyModel
from stack_of_tasks.ui.traits_mapping.custom_widgets.object_dropbown import ObjectModel

SOLVER = [InverseJacobianSolver, OSQPSolver]


class SolverConfig(ta.HasTraits):
    solver_model = ObjectModel(data=[OSQPSolver, InverseJacobianSolver])

    solver_cls: Solver = ta.Type(klass=Solver)
    _solver = ta.Instance(Solver)

    @ta.on_trait_change("solver_cls", post_init=False)
    def _solver_changed(self):
        self._solver = self.solver_cls(self.n, self.st)


class HierarchyController:
    pass


class DummyTask(Task):
    def compute(self):
        return None


class Controller(ta.HasTraits):
    tasks_model = ObjectModel(data=[PositionTask, ParallelTask])

    solver = SolverConfig()

    st = TaskHierarchy()
    task = PositionTask(
        None,
        None,
        TaskSoftnessType.linear,
    )
    task2 = ParallelTask(None, None, TaskSoftnessType.linear)

    with st.new_level() as level:
        task.observe(print, "*")
        level.append(task)
        level.append(task2)
    n = 8

    st_model = TaskHierarchyModel(st)


if __name__ == "__main__":
    c = Controller()
    import os
    from sys import argv

    os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"
    QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps)

    app = QApplication(argv)

    mw = Ui()
    mw.tab_widget.hierarchy.set_task_hierarchy_model(c.st_model)
    mw.run_Button.clicked.connect(lambda: c.task.trait_set(weight=random()))
    app.exec()

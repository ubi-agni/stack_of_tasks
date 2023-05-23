#!/usr/bin/env python3

from __future__ import annotations

import os
from sys import argv
from threading import Event, Thread, current_thread

from typing import Any, Generic, List, Type, TypeVar

import numpy as np
import traits.api as ta
from PyQt5 import QtCore
from PyQt5.QtCore import QCoreApplication
from PyQt5.QtWidgets import QApplication

import rospy

from stack_of_tasks.ref_frame import Offset, RefFrame
from stack_of_tasks.ref_frame.frames import Origin, RobotRefFrame
from stack_of_tasks.robot_model.jointstate_publisher import JointStatePublisher
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.robot_model.robot_state import RobotState
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.solver.OSQPSolver import OSQPSolver
from stack_of_tasks.tasks.Eq_Tasks import OrientationTask, ParallelTask, PositionTask
from stack_of_tasks.tasks.Task import Task, TaskSoftnessType
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy
from stack_of_tasks.ui.mainwindow import Ui
from stack_of_tasks.ui.model.object_model import ObjectModel, display_cls_name
from stack_of_tasks.ui.model.task_hierarchy import TaskHierarchyModel
from stack_of_tasks.ui.model_mapping import ClassKey, InjectionArg, ModelMapping
from stack_of_tasks.ui.traits_mapping.bindings import (
    TraitObjectModelBinder,
    TraitWidgetBinding,
    trait_widget_binding,
)

SOLVER = [InverseJacobianSolver, OSQPSolver]


T = TypeVar("T")


class DependencyInjection:
    mapping = {}

    @staticmethod
    def inject(arg_vector):
        injected = {}

        for arg in arg_vector.keys():
            if isinstance(arg_vector[arg], InjectionArg):
                injected[arg] = DependencyInjection.mapping[arg]
            else:
                injected[arg] = arg_vector[arg]

        return injected

    @staticmethod
    def create_instance(instance_cls: Type[T], kwargs: dict) -> T:
        # inject arguments
        return instance_cls(**DependencyInjection.inject(kwargs))


class Worker:
    def __init__(self, controller: Controller, rate: int) -> None:
        self.controller = controller

        self.warmstart = None
        self.rate = rate
        self.rrate = rospy.Rate(rate)

    def reset(self):
        self.warmstart = None

    def calculate(self) -> Any:
        lb = np.maximum(
            -0.01,
            (
                self.controller.robot_model.mins * 0.95
                - self.controller.robot_state.joint_values
            )
            / self.rate,
        )
        ub = np.minimum(
            0.01,
            (
                self.controller.robot_model.maxs * 0.95
                - self.controller.robot_state.joint_values
            )
            / self.rate,
        )

        dq = self.controller.solver.solve(lb, ub, warmstart=self.warmstart)

        if dq is not None:
            self.controller.robot_state.actuate(dq)
            self.warmstart = dq
        else:
            self.warmstart = None

    def wait(self):
        self.rrate.sleep()


class SolverThread(Thread):
    def __init__(self, worker: Worker) -> None:
        super().__init__(name="WorkerThread", daemon=False)

        self.stop_event = Event()
        self.worker = worker

    def run(self) -> None:
        self.worker.reset()
        while True:
            if self.stop_event.is_set():
                break
            self.worker.calculate()
            self.worker.wait()


class Controller(ta.HasTraits):
    solver_cls: Type[Solver] = ta.Type(klass=Solver)
    solver: Solver = ta.Instance(Solver)

    def __init__(self):
        super(Controller).__init__()

        self.robot_model = RobotModel()
        self.robot_state = RobotState(self.robot_model)
        self.jst = JointStatePublisher(self.robot_state)

        self.task_hierarchy = TaskHierarchy()

        self.worker = Worker(self, 50)
        self.thread: SolverThread = None

        self.on_trait_change(self._solver_changed, "solver_cls")

    def _solver_changed(self):
        self.solver: Solver = self.solver_cls(self.robot_model.N, self.task_hierarchy)
        self.solver.stack_changed()

    def toggle_solver_state(self):
        if self.thread is None:
            self.thread = SolverThread(self.worker)
            self.thread.start()
            return True
        else:
            self.thread.stop_event.set()
            self.thread.join()
            self.thread = None
            return False

    def is_thread_running(self):
        return self.thread is not None and self.thread.is_alive()


class Main(ta.HasTraits):
    def __init__(self):
        super().__init__()

        RefFrame.add_class_trait("display_name", ta.Str)
        RobotRefFrame.class_traits()["robot_state"].injected = "robot_state"
        Task.add_class_trait("display_name", ta.Str)

        self.controller = Controller()

        DependencyInjection.mapping["robot_state"] = self.controller.robot_state

        # QT-Models
        self.link_model = ObjectModel(
            data=[*self.controller.robot_model.links.keys()]
        )  # all robot links
        RobotRefFrame.class_traits()["link_name"].selection_model = self.link_model

        self.solver_cls_model = ObjectModel(
            data=[OSQPSolver, InverseJacobianSolver], disp_func=display_cls_name
        )

        self.task_class_model = ObjectModel(
            data=[PositionTask, ParallelTask, OrientationTask], disp_func=display_cls_name
        )
        ModelMapping.add_mapping(ClassKey(Task), self.task_class_model)

        self.refs_cls_model = ObjectModel(
            data=[Origin, Offset, RobotRefFrame], disp_func=display_cls_name
        )
        ModelMapping.add_mapping(ClassKey(RefFrame), self.refs_cls_model)

        self.refs: List[RefFrame]
        self.add_trait("refs", ta.List(RefFrame))
        self.refs_model = ObjectModel(disp_func=lambda x: x.display_name)
        ModelMapping.add_mapping(RefFrame, self.refs_model)

        TraitObjectModelBinder(self, "refs", self.refs_model)

        self.task_hierachy_model = TaskHierarchyModel(self.controller.task_hierarchy)

        def _reset_th(evt):
            print(evt)
            self.task_hierachy_model.beginResetModel()
            self.task_hierachy_model.endResetModel()

        self.controller.task_hierarchy.observe(_reset_th, "*")

    def setup(self):
        o = Origin(display_name="Origin")

        off = Offset(frame=o, display_name="Offset")

        frame = RobotRefFrame(
            self.controller.robot_state, "panda_hand_tcp", display_name="hand"
        )

        self.refs.extend([o, off, frame])

        with self.controller.task_hierarchy.new_level() as level:
            level.append(PositionTask(off, frame, TaskSoftnessType.linear))

    def new_ref(self, cls, args):
        new_ref = DependencyInjection.create_instance(cls, args)
        self.refs.append(new_ref)

    def new_task(self, cls, args):
        new_task = DependencyInjection.create_instance(cls, args)

        with self.controller.task_hierarchy.new_level() as l:
            l.append(new_task)

    def teardown(self):
        if self.controller.is_thread_running():
            self.controller.toggle_solver_state()


def main():
    os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"
    QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps)

    main_app = Main()
    main_app.setup()

    app = QApplication(argv)
    ui_window = Ui()

    app.aboutToQuit.connect(main_app.teardown)

    TraitWidgetBinding(
        main_app.controller, "solver", ui_window.tab_widget.solver.edit_solver, "trait_object"
    )
    trait_widget_binding(
        main_app.controller, "solver_cls", ui_window.tab_widget.solver.solverClassComboBox
    )

    ui_window.tab_widget.solver.solverClassComboBox.setModel(main_app.solver_cls_model)

    ui_window.tab_widget.hierarchy.set_task_hierarchy_model(main_app.task_hierachy_model)
    ui_window.tab_widget.refs.set_model(main_app.refs_model)
    ui_window.tab_widget.refs.new_ref_signal.connect(main_app.new_ref)

    ui_window.tab_widget.hierarchy.new_task_signal.connect(main_app.new_task)

    def button():
        state = main_app.controller.toggle_solver_state()
        if state:
            ui_window.run_Button.setText("Stop")
        else:
            ui_window.run_button.setText("Start")

    ui_window.run_Button.clicked.connect(button)

    app.exec()


if __name__ == "__main__":
    rospy.init_node("ik")
    main()

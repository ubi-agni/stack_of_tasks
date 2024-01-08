#!/usr/bin/env python3

from __future__ import annotations

import logging
from sys import argv
from threading import Event, Thread

from typing import Any, Generic, List, Type, TypeVar

import numpy as np
import traits.api as ta
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QFileDialog
from traits.trait_notifiers import set_ui_handler

import rospy

from stack_of_tasks.config.load_safe import LoadSafe
from stack_of_tasks.logger import fix_rospy_logging, sot_logger
from stack_of_tasks.marker import IAMarker, MarkerRegister
from stack_of_tasks.marker.marker_server import MarkerServer
from stack_of_tasks.marker.trait_marker import FullMovementMarker
from stack_of_tasks.ref_frame import MarkerFrame, RefFrame, RefFrameRegister
from stack_of_tasks.ref_frame.frames import Origin, RobotRefFrame
from stack_of_tasks.robot_model.actuators import Actuator, DummyPublisherActuator
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.robot_model.robot_state import RobotState
from stack_of_tasks.solver import SolverRegister
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.tasks import Task, TaskHierarchy, TaskRegister, TaskSoftnessType
from stack_of_tasks.tasks.Eq_Tasks import OrientationTask, PositionTask
from stack_of_tasks.ui.mainwindow import Ui
from stack_of_tasks.ui.model.object_model import ObjectModel
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.property_tree.prop_tree import SOT_Model
from stack_of_tasks.ui.traits_mapping.bindings import (
    TraitObjectModelBinder,
    TraitWidgetBinding,
    trait_widget_binding,
)
from stack_of_tasks.ui.utils.dependency_injection import DependencyInjection
from stack_of_tasks.utils.traits import BaseSoTHasTraits

logger = sot_logger.getChild("main")


class Worker:
    def __init__(self, controller: Controller, rate: int) -> None:
        self.controller = controller

        self.warmstart = None
        self.rate = rate
        self.rrate = rospy.Rate(rate)

    def reset(self):
        self.warmstart = None

    def calculate(self) -> Any:
        c = self.controller
        c.robot_state.update()  # read current joint values
        m = c.robot_model
        lb = np.maximum(-m.vmaxs / self.rate, 0.95 * m.mins - c.robot_state.joint_values)
        ub = np.minimum(+m.vmaxs / self.rate, 0.95 * m.maxs - c.robot_state.joint_values)
        dq = c.solver.solve(lb, ub, warmstart=self.warmstart)

        if dq is not None:
            c.actuator.actuate(dq)
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


class Controller(BaseSoTHasTraits):
    solvercls: Type[Solver] = ta.Type(klass=Solver, value=None)
    solver: Solver = ta.Instance(klass=Solver)

    def __init__(self, *, robot_model: RobotModel = None, actuator: Actuator = None):
        super().__init__()

        self.robot_model = robot_model or RobotModel()
        self.robot_state = RobotState(self.robot_model)
        self.actuator = actuator or DummyPublisherActuator(self.robot_state)

        self.task_hierarchy = TaskHierarchy()

        self.worker = Worker(self, 50)
        self.thread: SolverThread = None

    @ta.observe("solvercls", post_init=True)
    def _solver_changed(self, evt):
        self.solver: Solver = self.solvercls(self.robot_model.N, self.task_hierarchy)
        self.solver.tasks_changed()

    def toggle_solver_state(self):
        if self.thread is None:
            self.solver.tasks_changed()
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


class Main(BaseSoTHasTraits):
    def __init__(self):
        super().__init__()

        self.controller = Controller()
        self.marker_server = MarkerServer()

        model = self.controller.robot_model
        IAMarker._default_frame_id = model.root_link
        RobotRefFrame.class_traits()["_robot_state"].injected = "robot_state"
        DependencyInjection.mapping["robot_state"] = self.controller.robot_state

        # QT-Models

        self.link_model = ObjectModel(data=[*model.links.keys()])  # all robot links

        RobotRefFrame.class_traits()[
            "link_name"
        ].enum_selection = self.link_model  # temp. workaround

        self.solver_cls_model = ObjectModel.from_class_register(SolverRegister)
        ModelMapping.add_mapping(ClassKey(Solver), self.solver_cls_model)

        self.task_class_model = ObjectModel.from_class_register(TaskRegister)
        ModelMapping.add_mapping(ClassKey(Task), self.task_class_model)

        self.refs_cls_model = ObjectModel.from_class_register(RefFrameRegister)
        ModelMapping.add_mapping(ClassKey(RefFrame), self.refs_cls_model)

        self.refs: List[RefFrame]
        self.add_trait("refs", ta.List(RefFrame))
        self.refs_model = ObjectModel()

        ModelMapping.add_mapping(RefFrame, self.refs_model)
        TraitObjectModelBinder(self, "refs", self.refs_model)

        self.marker: List[IAMarker]
        self.add_trait("marker", ta.List(IAMarker))

        self.observe(self._marker_list_changed, "marker:items")

        self.marker_model = ObjectModel()
        ModelMapping.add_mapping(IAMarker, self.marker_model)
        TraitObjectModelBinder(self, "marker", self.marker_model)

        self.marker_cls_model = ObjectModel.from_class_register(MarkerRegister)
        ModelMapping.add_mapping(ClassKey(IAMarker), self.marker_cls_model)

        self.task_hierachy_model = SOT_Model(self.controller.task_hierarchy)
        ModelMapping.add_mapping(Task, self.task_hierachy_model)
        self.controller.observe(
            lambda evt: self.task_hierachy_model.set_task_hierarchy(evt.new), "task_hierarchy"
        )

        self.residual_update_timer = QtCore.QTimer()
        self.residual_update_timer.timeout.connect(self.update_residuals)

    def update_residuals(self):
        for task in self.controller.task_hierarchy.all_tasks():
            task._residual_update = True

    def setup(self):
        rospy.sleep(0.1)  # wait for joint_states message
        self.controller.robot_state.update()

        eef = RobotRefFrame(self.controller.robot_state, "panda_hand_tcp")
        self.new_marker(FullMovementMarker, dict(name="pose", transform=eef.T))

        self.refs.extend([Origin(), goal := MarkerFrame(self.marker[-1]), eef])

        with self.controller.task_hierarchy.new_level() as level:
            level.append(PositionTask(goal, eef, TaskSoftnessType.linear))
            level.append(OrientationTask(goal, eef, TaskSoftnessType.linear))

    def new_ref(self, cls, args):
        new_ref = DependencyInjection.create_instance(cls, args)
        self.refs.append(new_ref)

    def new_task(self, cls, args):
        new_task = DependencyInjection.create_instance(cls, args)

        with self.controller.task_hierarchy.new_level() as l:
            l.append(new_task)

    def new_marker(self, cls, args):
        new_marker = DependencyInjection.create_instance(cls, args)
        self.marker.append(new_marker)
        self._add_marker_to_server(new_marker)

    def _marker_list_changed(self, evt):
        for x in evt.added:
            self._add_marker_to_server(x)
        else:
            for x in evt.removed:
                self.marker_server.remove(x)

    def _add_marker_to_server(self, marker):
        self.marker_server.add_marker(marker)

    def teardown(self):
        if self.controller.is_thread_running():
            self.controller.toggle_solver_state()

    def save_state(self):
        url, _ = QFileDialog.getSaveFileUrl(filter="YAML - Files (*.yml)")
        yml_str = LoadSafe.save_config(self.controller.task_hierarchy)

        with open(url.path(), "w") as f:
            f.write(yml_str)

    def load_state(self):
        url, _ = QFileDialog.getOpenFileUrl()
        with open(url.path(), "r") as f:
            yml_str = f.read()

        data = LoadSafe.load_config(yml_str)
        frames = data["frames"]
        marker = data["marker"]
        sot: dict = data["stack_of_tasks"]

        self.marker.extend(marker)
        self.refs.extend(frames)

        for k, level in sorted(sot.items()):
            with self.controller.task_hierarchy.new_level() as l:
                l.extend(level)

    def clear_data(self):
        for l in self.controller.task_hierarchy.levels:
            l.clear()
        self.controller.task_hierarchy.levels.clear()

        self.refs.clear()
        self.marker.clear()


def _ui_handler(handler, *args, **kwargs):
    logger.debug(f"handler {handler}, args {args}, kwargs {kwargs}")


def main():
    logger.info("create main")
    set_ui_handler(_ui_handler)

    main_app = Main()
    # main_app.setup()

    logger.info("create application ")
    app = QApplication(argv)
    ui_window = Ui()

    app.aboutToQuit.connect(main_app.teardown)

    TraitWidgetBinding(
        main_app.controller, "solver", ui_window.tab_widget.solver.edit_solver, "trait_object"
    )

    trait_widget_binding(
        main_app.controller,
        "solvercls",
        ui_window.tab_widget.solver.solverClassComboBox,
        set_trait_post=True,
    )

    ui_window.tab_widget.refs.new_ref_signal.connect(main_app.new_ref)
    ui_window.tab_widget.marker.new_marker_signal.connect(main_app.new_marker)

    def toggle_start_stop():
        if main_app.controller.toggle_solver_state():
            ui_window.run_Button.setText("Stop")
            main_app.residual_update_timer.start(100)
        else:
            ui_window.run_Button.setText("Start")
            main_app.residual_update_timer.stop()

    ui_window.run_Button.clicked.connect(toggle_start_stop)

    ui_window.save_action.triggered.connect(main_app.save_state)
    ui_window.load_action.triggered.connect(main_app.load_state)
    ui_window.new_action.triggered.connect(main_app.clear_data)

    app.exec()


if __name__ == "__main__":
    rospy.init_node("ik")
    fix_rospy_logging(sot_logger)
    sot_logger.setLevel(logging.DEBUG)
    main()

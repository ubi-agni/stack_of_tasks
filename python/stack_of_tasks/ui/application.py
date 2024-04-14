#!/usr/bin/env python3

from __future__ import annotations

import logging
from collections import namedtuple
from datetime import datetime
from pathlib import Path
from sys import argv
from threading import Event, Thread

from typing import Any, List, Tuple, Type, TypedDict

import numpy as np
import traits.api as ta
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QFileDialog
from traits.trait_notifiers import set_ui_handler

import rospy

from stack_of_tasks import syringe
from stack_of_tasks.config import Configuration, dump, load
from stack_of_tasks.controller import Controller
from stack_of_tasks.logger import fix_rospy_logging, sot_logger
from stack_of_tasks.marker import IAMarker, MarkerRegister
from stack_of_tasks.marker.marker_server import MarkerServer
from stack_of_tasks.ref_frame import RefFrame, RefFrameRegister
from stack_of_tasks.ref_frame.frames import RobotRefFrame
from stack_of_tasks.solver import AbstractSolver, SolverRegister
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.tasks import Task, TaskRegister
from stack_of_tasks.ui.mainwindow import Ui
from stack_of_tasks.ui.model.object_model import ObjectModel
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.proj_window import Ui_Project_Window
from stack_of_tasks.ui.property_tree.prop_tree import SOT_Model
from stack_of_tasks.ui.traits_mapping.bindings import (
    TraitObjectModelBinder,
    TraitWidgetBinding,
    trait_widget_binding,
)
from stack_of_tasks.ui.utils.dependency_injection import DependencyInjection
from stack_of_tasks.utils.traits import BaseSoTHasTraits

logger = sot_logger.getChild("main")

import json

import platformdirs


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


class TraitsEvent(QtCore.QEvent):
    _QT_TRAITS_EVENT = QtCore.QEvent.Type(QtCore.QEvent.registerEventType())

    def __init__(self, handler, args, kwargs):
        super().__init__(TraitsEvent._QT_TRAITS_EVENT)

        self.handler = handler
        self.args = args
        self.kwargs = kwargs


class EventProcessor(QtCore.QObject):
    def event(self, a0: TraitsEvent) -> bool:
        # print("process event thread ", threading.current_thread().native_id)
        if a0.type() == TraitsEvent._QT_TRAITS_EVENT:
            a0.handler(*a0.args, **a0.kwargs)
            return True
        return super().event(a0)

    def postEvent(self, handler, *args, **kwargs):
        QApplication.instance().postEvent(self, TraitsEvent(handler, args, kwargs))


class UI_Controller(Controller):
    def __init__(self, config: Configuration):
        super().__init__(config)

        self.worker = Worker(self, 50)
        self.thread: SolverThread = None

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


LATEST_PATH = platformdirs.user_cache_path("stack_of_tasks") / "latest.json"


class Logic_Main:

    latest: dict[Path, Tuple[str, datetime]]

    def __init__(self) -> None:

        self.ui = Ui_Project_Window()

        self.latest: dict[Path, List[str, datetime]] = {}
        self._load_latest()

        self.current_project = None

        self.ui.open_project_from_file.connect(self._open_from_file)
        self.ui.open_project.connect(self._open_recent)
        self.ui.new_project.connect(self.new_project)

        self._evt_processor = EventProcessor()
        set_ui_handler(self._evt_processor.postEvent)

        if len(self.latest) == 0:
            self.new_project()

    def _load_latest(self):
        if LATEST_PATH.exists():
            data = json.loads(LATEST_PATH.read_text())
            self.latest = {
                Path(k): [v[0], datetime.fromtimestamp(v[1])] for k, v in data.items()
            }

            self.ui.set_last_items(self.latest)

    def _save_latest(self):
        data = {k.as_posix(): (v[0], v[1].timestamp()) for k, v in self.latest.items()}
        LATEST_PATH.write_text(json.dumps(data))

    def _update_latest_project_list(self, url: Path, name: str = None):
        if url in self.latest:
            self.latest[url][1] = datetime.now()
        else:
            self.latest[url] = ("Project name", datetime.now())

        self._save_latest()

    # Projecct management

    def _safe(self, url: Path):
        yaml_str = dump(self.current_project.controller)
        url.write_text(yaml_str)

        self._update_latest_project_list(url)

    def _safe_as(self):
        url, _ = QFileDialog.getSaveFileName(filter="YAML - Files (*.yml)")
        self._safe(Path(url))

    def new_project(self):
        if self.current_project is not None:
            self.current_project.teardown()
            self.current_project.ui_window.close()
            del self.current_project

        from stack_of_tasks.robot_model.actuators import JointStatePublisherActuator
        from stack_of_tasks.solver.OSQPSolver import OSQPSolver

        config = Configuration(
            actuator_cls=JointStatePublisherActuator, solver_cls=OSQPSolver
        )
        self._create_project(config)
        self.ui.close()

    def _create_project(self, config):
        self.current_project = Logic_Project(config)

        self.current_project.ui_window.new_signal.connect(self.new_project)

        self.current_project.ui_window.open_file_signal.connect(self._open_from_file)
        self.current_project.ui_window.save_signal.connect(self._safe)
        self.current_project.ui_window.save_as_signal.connect(self._safe_as)

        self.current_project.ui_window.show()

    def _load_project(self, config_location: Path):
        config = load(config_location.read_text())
        self._update_latest_project_list(config_location)
        self._create_project(config)
        self.ui.close()

    def _open_recent(self, index: int):
        self._load_project(list(self.latest.keys())[index])

    def _open_from_file(self):
        name, _ = QFileDialog.getOpenFileName(filter="YAML - Files (*.yml)")
        if name != "":
            self._load_project(Path(name))


class Logic_Project(BaseSoTHasTraits):

    ref_objects: List[RefFrame] = ta.List(RefFrame)
    marker_objects: List[IAMarker] = ta.List(IAMarker)
    solver_cls = ta.Property()

    def __init__(self, config: Configuration):
        super().__init__()

        self.controller = UI_Controller(config)

        QApplication.instance().aboutToQuit.connect(self.teardown)

        self.marker_server = MarkerServer()
        IAMarker._default_frame_id = self.controller.robot_model.root_link

        ##
        self.ref_objects.extend(config.instancing_data.objects["frames"])
        self.marker_objects.extend(config.instancing_data.objects["marker"])

        # QT-Models

        # class models -> should be global state
        self.solver_cls_model = ObjectModel.from_class_register(SolverRegister)
        self.task_class_model = ObjectModel.from_class_register(TaskRegister)
        self.refs_cls_model = ObjectModel.from_class_register(RefFrameRegister)
        self.marker_cls_model = ObjectModel.from_class_register(MarkerRegister)

        # runtime models

        self.refs_model = ObjectModel()
        self.marker_model = ObjectModel()

        self.task_hierachy_model = SOT_Model(self.controller.task_hierarchy)

        # set all robot model links globally
        self.link_model = ObjectModel(
            data=[*self.controller.robot_model.links.keys()]
        )  # all robot links

        RobotRefFrame.class_traits()[
            "link_name"
        ].enum_selection = self.link_model  # temp. workaround

        ModelMapping.add_mapping(ClassKey(Solver), self.solver_cls_model)
        # ModelMapping.add_mapping(ClassKey(Actu), self.solver_cls_model)
        ModelMapping.add_mapping(ClassKey(Task), self.task_class_model)
        ModelMapping.add_mapping(ClassKey(RefFrame), self.refs_cls_model)
        ModelMapping.add_mapping(ClassKey(IAMarker), self.marker_cls_model)

        ModelMapping.add_mapping(RefFrame, self.refs_model)
        ModelMapping.add_mapping(IAMarker, self.marker_model)
        ModelMapping.add_mapping(Task, self.task_hierachy_model)

        TraitObjectModelBinder(self, "ref_objects", self.refs_model, True)
        TraitObjectModelBinder(self, "marker_objects", self.marker_model, True)

        self.controller.observe(
            lambda evt: self.task_hierachy_model.set_task_hierarchy(evt.new), "task_hierarchy"
        )

        self.ui_window = Ui()
        ### UI STuff

        TraitWidgetBinding(
            self.controller,
            "solver",
            self.ui_window.tab_widget.solver.edit_solver,
            "trait_object",
            set_post_init=True,
        )

        trait_widget_binding(
            self,
            "solver_cls",
            self.ui_window.tab_widget.solver.solverClassComboBox,
            set_widget_post=True,
        )

        self.ui_window.tab_widget.refs.new_ref_signal.connect(self.new_ref)
        self.ui_window.tab_widget.marker.new_marker_signal.connect(self.new_marker)
        self.ui_window.run_Button.clicked.connect(self.toggle_start_stop)

    def update_residuals(self):
        for task in self.controller.task_hierarchy.all_tasks():
            task._residual_update = True

    def _set_solver_cls(self, solver_cls: Type[AbstractSolver.Solver]):
        solver = solver_cls(task_hierarchy=self.controller.task_hierarchy)
        self.controller.solver = solver

    def _get_solver_cls(self):
        return type(self.controller.solver)

    def _solver_cls_changed(self, solver_cls: Type[Solver]):
        solver = solver_cls(self.controller.robot_model.N, self.controller.task_hierarchy)
        self.controller.solver = solver

    def new_ref(self, cls, args):
        new_ref = cls(**args)
        self.ref_objects.append(new_ref)

    def new_task(self, cls, args):
        new_task = DependencyInjection.create_instance(cls, args)

        with self.controller.task_hierarchy.new_level() as l:
            l.append(new_task)

    def new_marker(self, cls, args):
        new_marker = DependencyInjection.create_instance(cls, args)
        self.marker_objects.append(new_marker)
        self._add_marker_to_server(new_marker)

    @ta.observe("marker_objects:items")
    def _marker_list_changed(self, evt):
        for x in evt.added:
            self._add_marker_to_server(x)
        for x in evt.removed:
            self.marker_server.remove(x)

    def _add_marker_to_server(self, marker):
        self.marker_server.add_marker(marker)

    def teardown(self):
        if self.controller.is_thread_running():
            self.controller.toggle_solver_state()

    def clear_data(self):
        for l in self.controller.task_hierarchy.levels:
            l.clear()
        self.controller.task_hierarchy.levels.clear()

        self.ref_objects.clear()
        self.marker_objects.clear()

    def toggle_start_stop(self):
        if self.controller.toggle_solver_state():
            self.ui_window.run_Button.setText("Stop")
        else:
            self.ui_window.run_Button.setText("Start")


def main():
    rospy.init_node("ik")
    fix_rospy_logging(sot_logger)
    sot_logger.setLevel(logging.DEBUG)

    app = QApplication(argv)
    _ = Logic_Main()

    app.exec()


if __name__ == "__main__":
    main()

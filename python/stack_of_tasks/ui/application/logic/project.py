from __future__ import annotations

from pathlib import Path
from threading import Event, Thread

from typing import Any, List, Type

import traits.api as ta
from PyQt5 import sip

import rospy

from stack_of_tasks.config import Configuration
from stack_of_tasks.controller import Controller
from stack_of_tasks.marker import IAMarker, MarkerRegister, MarkerServer
from stack_of_tasks.plot import PlotCSV, PlotPublisher
from stack_of_tasks.ref_frame import RefFrame, RefFrameRegister
from stack_of_tasks.ref_frame.frames import RobotRefFrame
from stack_of_tasks.robot_model.actuators import Actuator, ActuatorRegister, DummyActuator
from stack_of_tasks.solver import AbstractSolver, SolverRegister
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.tasks import Task, TaskRegister
from stack_of_tasks.ui.application.widgets.project_window import ProjectUI
from stack_of_tasks.ui.model.object_model import ObjectModel
from stack_of_tasks.ui.model.sot_model import SOT_Model
from stack_of_tasks.ui.model_mapping import ClassKey, ModelMapping
from stack_of_tasks.ui.traits_mapping.bindings import (
    SOT_Model_Binder,
    TraitObjectModelBinder,
    trait_widget_binding,
)
from stack_of_tasks.ui.utils.dependency_injection import DependencyInjection
from stack_of_tasks.utils.traits import BaseSoTHasTraits


## Do this with qthread?
class WorkerThread(Thread):

    _current_worker: WorkerThread = None
    _stop_event = Event()

    def __init__(self, controller: Controller) -> None:
        super().__init__(daemon=True, name="SolverThread")

        self._warmstart = None
        self._rosrate = rospy.Rate(controller.rate)
        self._c = controller

        self._c.solver.tasks_changed()

    @classmethod
    def toggle_running(cls, controller: Controller):

        if cls._current_worker is None:
            cls._current_worker = WorkerThread(controller)
            cls._current_worker.start()
            return True
        else:
            cls._stop_event.set()
            cls._current_worker.join()
            cls._current_worker = None
            cls._stop_event.clear()
            return False

    @classmethod
    def is_running(cls):
        return not cls._stop_event.is_set() and cls._current_worker is not None

    def run(self) -> None:

        while not self._stop_event.is_set():
            self._warmstart = self._c.control_step(warmstart=self._warmstart)
            self._rosrate.sleep()


class Logic_Project(BaseSoTHasTraits):

    url = ta.Property()

    ref_objects: List[RefFrame] = ta.List(RefFrame)
    marker_objects: List[IAMarker] = ta.List(IAMarker)
    solver_cls = ta.Property()
    actuator_cls = ta.Property()

    controller = ta.Instance(Controller)

    def __init__(self, config: Configuration):
        super().__init__()

        self.controller = Controller(config)

        self.marker_server = MarkerServer()
        IAMarker._default_frame_id = self.controller.robot_model.root_link

        ##
        objects = config.objects
        self.ref_objects.extend(objects["frames"])
        self.marker_objects.extend(objects["marker"])

        # QT-Models

        # class models -> should be global state
        self.solver_cls_model = ObjectModel.from_class_register(SolverRegister)
        self.actuator_cls_model = ObjectModel.from_class_register(ActuatorRegister)
        self.task_class_model = ObjectModel.from_class_register(TaskRegister)
        self.refs_cls_model = ObjectModel.from_class_register(RefFrameRegister)
        self.marker_cls_model = ObjectModel.from_class_register(MarkerRegister)

        # runtime models

        self.refs_model = ObjectModel()
        self.marker_model = ObjectModel()

        self.task_hierachy_model = SOT_Model()
        SOT_Model_Binder(self.task_hierachy_model, self.controller.task_hierarchy)

        # set all robot model links globally
        self.link_model = ObjectModel(data=self.controller.robot_model.link_names)  # all robot links
        RobotRefFrame.class_traits()["link_name"].enum_selection = self.link_model  # temp. workaround

        ModelMapping.add_mapping(ClassKey(Solver), self.solver_cls_model)
        ModelMapping.add_mapping(ClassKey(Actuator), self.actuator_cls_model)
        ModelMapping.add_mapping(ClassKey(Task), self.task_class_model)
        ModelMapping.add_mapping(ClassKey(RefFrame), self.refs_cls_model)
        ModelMapping.add_mapping(ClassKey(IAMarker), self.marker_cls_model)

        ModelMapping.add_mapping(RefFrame, self.refs_model)
        ModelMapping.add_mapping(IAMarker, self.marker_model)

        ModelMapping.add_mapping(Task, self.task_hierachy_model)

        TraitObjectModelBinder(self, "ref_objects", self.refs_model, True)
        TraitObjectModelBinder(self, "marker_objects", self.marker_model, True)

        self.ui = ProjectUI()

        self.ui.settings_tab.settings.set_trait_object(self.controller)

        trait_widget_binding(
            self,
            "solver_cls",
            self.ui.settings_tab.solverClassComboBox,
            set_widget_post=True,
        )
        self.ui.settings_tab.edit_solver.set_trait_object(self.controller.solver)

        trait_widget_binding(
            self,
            "actuator_cls",
            self.ui.settings_tab.actuatorClassComboBox,
            set_widget_post=True,
        )
        self.ui.settings_tab.edit_actuator.set_trait_object(self.controller.actuator)

        self.plot = PlotPublisher(self.controller, frames=self.ref_objects)

        self.ui.refs_tab.new_ref_signal.connect(self.new_ref)
        self.ui.marker_tab.new_marker_signal.connect(self.new_marker)
        self.ui.run_Button.clicked.connect(self.toggle_start_stop)

        self.url = None

    def _set_url(self, url: Path):
        self._url = url
        self.ui.setWindowTitle(f"Stack of Tasks - {self.url.name if self.url else 'New Project'}")

    def _get_url(self):
        return self._url

    def update_residuals(self):
        for task in self.controller.task_hierarchy.all_tasks():
            task._residual_update = True

    def _set_solver_cls(self, solver_cls: Type[AbstractSolver.Solver]):
        if not isinstance(self.controller.solver, solver_cls):
            self.controller.solver = solver_cls(task_hierarchy=self.controller.task_hierarchy)
        self.ui.settings_tab.edit_solver.set_trait_object(self.controller.solver)

    def _get_solver_cls(self):
        return type(self.controller.solver)

    def _set_actuator_cls(self, actuator_cls: Type[Actuator]):
        if not isinstance(self.controller.actuator, actuator_cls):
            try:
                self.controller.actuator = actuator_cls()
            except Exception as e:  # on failure, fallback to DummyActuator
                rospy.logerr(f"Failed creating {actuator_cls.__name__} with {e.__class__.__name__}: {e}")
                self.ui.settings_tab.actuatorClassComboBox.set_current_object(DummyActuator)
        self.ui.settings_tab.edit_actuator.set_trait_object(self.controller.actuator)

    def _get_actuator_cls(self):
        return type(self.controller.actuator)

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

    @ta.observe("marker_objects:items")
    def _marker_list_changed(self, evt):
        for x in evt.added:
            self.marker_server.add_marker(x)
        for x in evt.removed:
            self.marker_server.remove(x)

    def toggle_start_stop(self):
        if WorkerThread.toggle_running(self.controller):
            self.ui.run_Button.setText("Stop")
        else:
            self.ui.run_Button.setText("Start")

    ## logic deletion procedure

    def teardown(self):

        self.clear_data()

        if not sip.isdeleted(self.ui):
            self.ui.close()
            self.ui.deleteLater()

        if WorkerThread.is_running():
            WorkerThread.toggle_running(None)

    def clear_data(self):

        self.ref_objects.clear()
        self.marker_objects.clear()

        ModelMapping.clear_mapping()

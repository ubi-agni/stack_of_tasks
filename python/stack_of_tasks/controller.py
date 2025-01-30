from typing import Callable, Type

import numpy as np
import traits.api as ta

import rospy

from stack_of_tasks import syringe
from stack_of_tasks.config import Configuration
from stack_of_tasks.robot_model.actuators import Actuator
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.robot_model.robot_state import RobotState
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.tasks import TaskHierarchy
from stack_of_tasks.utils.traits import BaseSoTHasTraits


class Controller(BaseSoTHasTraits):
    robot_model: RobotModel = ta.Instance(RobotModel, visible=False)
    robot_state: RobotState = ta.Instance(RobotState, visible=False)

    solver: Solver = ta.Instance(Solver, visible=False)
    actuator: Actuator = ta.Instance(Actuator, visible=False)

    task_hierarchy: TaskHierarchy = ta.Instance(TaskHierarchy, visible=False)

    rate = ta.Range(1.0, 500, value=50, step=1.0)
    _updated = ta.Event()  # event triggered after each control step

    def __init__(self, config: Configuration) -> None:
        super().__init__()

        # robotmodel
        if "robot_model_param" in config.settings:
            self.robot_model = RobotModel(config.settings.pop("robot_model_param"))
        else:
            self.robot_model = RobotModel()
        # robotstate
        self.robot_state = RobotState(self.robot_model)

        # set global data
        syringe[RobotModel] = self.robot_model
        syringe[RobotState] = self.robot_state

        self.actuator: Actuator = config.settings.pop("actuator").instance
        self.solver: Solver = config.settings.pop("solver").instance

        # settings
        for k, v in config.settings.items():
            setattr(self, k, v)

        # collection of tasks
        self.task_hierarchy = config.stack_of_tasks
        self.solver.set_task_hierarchy(self.task_hierarchy)

        self.dq = None

    def control_loop(self, stopping_condition: Callable[[], bool]):
        rate = rospy.Rate(self.rate)
        warmstart_dq = None
        self.solver.tasks_changed()

        while not stopping_condition():
            warmstart_dq = self.control_step(warmstart_dq)
            rate.sleep()

    def control_step(self, warmstart):
        self.robot_state.update()  # read current joint values
        m = self.robot_model
        dq_max = m.vmaxs / self.rate
        lb = np.maximum(-dq_max, np.minimum(0.0, m.mins - self.robot_state.joint_values))
        ub = np.minimum(+dq_max, np.maximum(0.0, m.maxs - self.robot_state.joint_values))
        self.dq = dq = self.solver.solve(lb, ub, warmstart=warmstart)

        if dq is not None:
            self.actuator.actuate(dq)
            self._updated = True
        else:
            pass

        return dq

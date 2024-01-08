from typing import Callable, Type

import numpy as np
import traits.api as ta

import rospy

from stack_of_tasks import syringe
from stack_of_tasks.config import Configuration
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.robot_model.robot_state import RobotState
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.tasks import TaskHierarchy
from stack_of_tasks.utils.traits import BaseSoTHasTraits


class Controller(BaseSoTHasTraits):
    config: Configuration = ta.Instance(Configuration)

    robot_model: RobotModel = ta.Instance(RobotModel)
    robot_state: RobotState = ta.Instance(RobotState)

    solver: Solver = ta.Instance(Solver)

    task_hierarchy: TaskHierarchy = ta.Instance(TaskHierarchy)

    def __init__(self, config: Configuration) -> None:
        super().__init__()
        self.config = config

        # robotmodel
        self.robot_model = RobotModel()
        # robotstate
        self.robot_state = RobotState(self.robot_model)

        # set global data
        syringe[RobotModel] = self.robot_model
        syringe[RobotState] = self.robot_state

        self.actuator = self.config.parameter.actuator_cls(
            **self.config.parameter.actuator_parameter
        )

        # collection of tasks
        self.task_hierarchy = self.config.instancing_data.stack_of_tasks
        print(self.task_hierarchy)
        self.solver = self.config.parameter.solver_cls(
            self.robot_model.N, self.task_hierarchy, **self.config.parameter.solver_parameter
        )

    def control_loop(self, stopping_condition: Callable[[], bool], rate: int):
        rrate = rospy.Rate(rate)
        warmstart_dq = None
        self.solver.tasks_changed()

        while not stopping_condition():
            warmstart_dq = self.control_step(rate, warmstart_dq)
            rrate.sleep()

    def control_step(self, rate, warmstart):
        self.robot_state.update()  # read current joint values
        m = self.robot_model
        lb = np.maximum(-m.vmaxs / rate, 0.95 * m.mins - self.robot_state.joint_values)
        ub = np.minimum(+m.vmaxs / rate, 0.95 * m.maxs - self.robot_state.joint_values)
        dq = self.solver.solve(lb, ub, warmstart=warmstart)

        if dq is not None:
            self.actuator.actuate(dq)
        else:
            print("dq is none")

        return dq

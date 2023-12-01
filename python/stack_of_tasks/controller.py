from typing import Callable, Type

import numpy as np
import traits.api as ta

import rospy

from stack_of_tasks.robot_model.actuators import DummyPublisherActuator
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.robot_model.robot_state import RobotState
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy
from stack_of_tasks.utils.traits import BaseSoTHasTraits


class Controller(BaseSoTHasTraits):
    robot_model: RobotModel = ta.Instance(RobotModel)
    robot_state: RobotState = ta.Instance(RobotState)

    solver: Solver = ta.Instance(Solver)
    task_hierarchy: TaskHierarchy = ta.Instance(TaskHierarchy)

    def __init__(self, solver: Type[Solver], **solver_args) -> None:
        # robotmodel
        self.robot_model = RobotModel()

        # robotstate
        self.robot_state = RobotState(self.robot_model)

        super().__init__()

        self.actuator = DummyPublisherActuator(self.robot_state)

        # collection of tasks
        self.task_hierarchy = TaskHierarchy()
        self.solver = solver(self.robot_model.N, self.task_hierarchy, **solver_args)

    @ta.observe("task_hierarchy.stack_changed")
    def _stack_change(self, evt):
        self.solver.stack_changed()

    def control_loop(self, stopping_condition: Callable[[], bool], rate: int):
        rrate = rospy.Rate(rate)
        warmstart_dq = None
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

from typing import Callable, Type

import numpy as np
import traits.api as ta

import rospy

from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.robot_model.robot_state import RobotState
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy
from stack_of_tasks.utils.traits import BaseSoTHasTraits


class Controller(BaseSoTHasTraits):
    robot_model: RobotModel = ta.Instance(RobotModel)
    robot_state: RobotState = ta.Instance(RobotState)

    solver: Solver = ta.Instance(Solver)
    hierarchy: TaskHierarchy = ta.Instance(TaskHierarchy)

    def __init__(self, solver: Type[Solver], **solver_args) -> None:
        # robotmodel
        self.robot_model = RobotModel()

        # robotstate
        self.robot_state = RobotState(self.robot_model)

        super().__init__()

        # collection of tasks
        self.hierarchy = TaskHierarchy()
        self.solver = solver(self.robot_model.N, self.hierarchy, **solver_args)

    @ta.observe("hierarchy.stack_changed")
    def _stack_change(self, evt):
        self.solver.stack_changed()

    def control_loop(self, condition: Callable[[], bool], rate: int, invert_condition=False):
        _condition = lambda: not condition() if invert_condition else condition

        rrate = rospy.Rate(rate)

        warmstart_dq = None
        while _condition():
            warmstart_dq = self.control_step(rate, warmstart_dq)
            rrate.sleep()

    def control_step(self, rate, warmstart):
        lb = np.maximum(
            -0.01, (self.robot_model.mins * 0.95 - self.robot_state.joint_values) / rate
        )
        ub = np.minimum(
            0.01, (self.robot_model.maxs * 0.95 - self.robot_state.joint_values) / rate
        )

        dq = self.solver.solve(lb, ub, warmstart=warmstart)

        if dq is not None:
            self.robot_state.actuate(dq)
        else:
            print("dq is none")

        return dq

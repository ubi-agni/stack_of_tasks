from typing import Any

import numpy as np
import osqp
import scipy.sparse as snp

from stack_of_tasks import syringe
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.solver.HQPSolver import HqpSolver
from stack_of_tasks.tasks import EqTask
from stack_of_tasks.tasks.hierarchy import TaskHierarchy


class OSQPSolver(HqpSolver):
    @syringe.inject
    def __init__(self, robot_model: RobotModel, task_hierarchy=None, **options) -> None:
        super().__init__(robot_model, task_hierarchy, **options)
        self.slack_joint_matrix = None

    def _create_matrix(self):
        super()._create_matrix()
        self.slack_joint_matrix = np.identity(
            self.m_slacks + self.N,
        )

    def _interpret_solution(self, level_index, solution: object) -> Any:
        if solution.info.status_val != 1:
            return None

        dq = solution.x[: self.N]

        for task in self._task_hierarchy[level_index]:
            b = task.A.dot(dq)
            self._add_nullspace(task.A, b)

        return dq

    def _solve(self, warmstart, **options) -> Any:
        P = snp.csc_matrix(self.objective_matrix[: self.N + self.slacks, : self.N + self.slacks])
        q = self.objective_vector[: self.N + self.slacks]

        l = np.r_[
            self.joints_slack_lower[: self.N + self.slacks],
            self.nullspace_bound[: self.ns_rows],
            self.ieq_lower[: self.ieq_rows],
            self.eq_bound[: self.eq_rows],
            self.sieq_bound[: self.sieq_rows],
        ]
        u = np.r_[
            self.joints_slack_upper[: self.N + self.slacks],
            self.nullspace_bound[: self.ns_rows],
            self.ieq_upper[: self.ieq_rows],
            self.eq_bound[: self.eq_rows],
            np.full((self.sieq_rows,), np.inf),
        ]

        A = snp.vstack(
            [
                self.slack_joint_matrix[: self.N + self.slacks, : self.N + self.slacks],
                self.nullspace_matrix[: self.ns_rows, : self.N + self.slacks],
                self.ieq_matrix[: self.ieq_rows, : self.N + self.slacks],
                self.eq_matrix[: self.eq_rows, : self.N + self.slacks],
                self.sieq_matrix[: self.sieq_rows, : self.N + self.slacks],
            ],
            format="csc",
        )

        solver = osqp.OSQP()
        solver.setup(P=P, q=q, A=A, l=l, u=u, verbose=False, **options)
        if warmstart is not None:
            solver.warm_start(x=np.r_[warmstart, np.full(q.size - warmstart.size, 0)])

        solution = solver.solve()
        return solution

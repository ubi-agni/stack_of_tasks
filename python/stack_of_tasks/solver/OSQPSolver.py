from typing import Any

import numpy as np
import osqp
import scipy.sparse as snp

from stack_of_tasks.solver.HQPSolver import HqpSolver
from stack_of_tasks.tasks.Task import EqTask
from stack_of_tasks.utils import prettyprintMatrix

pp = lambda x: print(prettyprintMatrix(x))


class OSQPSolver(HqpSolver):
    def __init__(self, number_of_joints, stack_of_tasks, **options) -> None:
        super().__init__(number_of_joints, stack_of_tasks, **options)
        self.slack_joint_matrix = None

    def _create_matrix(self):
        super()._create_matrix()
        self.slack_joint_matrix = np.identity(
            self.m_slacks + self.N,
        )

    def _interpret_solution(self, level_index, solution: object) -> Any:
        if solution.info.status_val != 1:
            print("not solved")
            return None

        dq = solution.x[: self.N]

        for task in self._stack_of_tasks[level_index]:
            b = task.A.dot(dq)
            if isinstance(task, EqTask):

                task.residual = b - task.bound
                task.violation = ~np.isclose(task.residual, b)
            else:
                task.residual = None
                task.violation = (task.lower_bound < b) & (b < task.upper_bound)

            self._add_nullspace(task.A, b)

            # TODO importance

        return dq

    def _solve(self, warmstart, **options) -> Any:
        P = snp.csc_matrix(
            self.objective_matrix[: self.N + self.slacks, : self.N + self.slacks]
        )
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

#!/usr/bin/env python3


from typing import Any

import cvxopt
import numpy as np

from stack_of_tasks.solver.HQPSolver import HqpSolver
from stack_of_tasks.tasks.Task import EqTask


class CVXOPTSolver(HqpSolver):
    def __init__(self, number_of_joints, stack_of_tasks, **options) -> None:
        super().__init__(number_of_joints, stack_of_tasks, **options)
        self.slack_joint_matrix = None

    def _create_matrix(self):
        super()._create_matrix()
        self.slack_joint_matrix = np.identity(self.m_slacks + self.N)

    def _interpret_solution(self, level_index, solution: object) -> Any:
        dq = np.asarray(solution["x"]).reshape(-1)[: self.N]

        print(dq, solution["x"][self.N :])

        for task in self._stack_of_tasks.levels[level_index]:
            b = task.A.dot(dq)
            if isinstance(task, EqTask):
                task.residual = b - task.bound
                task.violation = ~np.isclose(task.residual, b)
            else:
                task.residual = None
                task.violation = (task.lower_bound < b) & (b < task.upper_bound)

            self._add_nullspace(task.A, b)

        return dq

    def _solve(self, warmstart, **options) -> Any:
        P = self.objective_matrix[: self.N + self.slacks, : self.N + self.slacks]
        q = self.objective_vector[: self.N + self.slacks]

        A = np.r_[
            self.nullspace_matrix[: self.ns_rows, : self.N + self.slacks],
            self.eq_matrix[: self.eq_rows, : self.N + self.slacks],
        ]
        b = np.r_[
            self.nullspace_bound[: self.ns_rows],
            self.eq_bound[: self.eq_rows],
        ]

        G = np.r_[
            self.slack_joint_matrix[: self.N + self.slacks, : self.N + self.slacks],
            -self.slack_joint_matrix[: self.N + self.slacks, : self.N + self.slacks],
            -self.sieq_matrix[: self.sieq_rows, : self.N + self.slacks],
            self.ieq_matrix[: self.ieq_rows, : self.N + self.slacks],
            -self.ieq_matrix[: self.ieq_rows, : self.N + self.slacks],
        ]

        h = np.r_[
            self.joints_slack_upper[: self.N + self.slacks],
            -self.joints_slack_lower[: self.N + self.slacks],
            -self.sieq_bound[: self.sieq_rows],
            self.ieq_upper[: self.ieq_rows],
            -self.ieq_lower[: self.ieq_rows],
        ]
        mask = np.less(h, np.inf)
        G = G[mask]
        h = h[mask]

        initvals = {}
        if warmstart is not None:
            initvals["x"] = cvxopt.matrix(np.r_[warmstart, np.full(0, self.slacks)])

        sol = cvxopt.solvers.qp(
            cvxopt.matrix(np.array(P)),
            cvxopt.matrix(np.array(q)),
            cvxopt.matrix(np.array(G)),
            cvxopt.matrix(np.array(h)),
            cvxopt.matrix(np.array(A)),
            cvxopt.matrix(np.array(b)),
            initvals=initvals,
            verbose=False,
        )
        return sol

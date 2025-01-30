#!/usr/bin/env python3


from typing import Any

import cvxopt
import numpy as np

from stack_of_tasks import syringe
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.solver.HQPSolver import HqpSolver
from stack_of_tasks.tasks.hierarchy import TaskHierarchy


class CVXOPTSolver(HqpSolver):
    @syringe.inject
    def __init__(self, robot_model: RobotModel, task_hierarchy: TaskHierarchy = None, **options) -> None:
        super().__init__(robot_model, task_hierarchy, **options)
        self.slack_joint_matrix = None

    def _create_matrix(self):
        super()._create_matrix()
        self.slack_joint_matrix = np.identity(self.m_slacks + self.N)

    def _interpret_solution(self, level_index, solution: object) -> Any:
        dq = np.asarray(solution["x"]).reshape(-1)[: self.N]

        for task in self._task_hierarchy.levels[level_index]:
            b = task.A.dot(dq)
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
            options=dict(show_progress=False),
        )
        return sol

#!/usr/bin/env python3

from typing import Any

import numpy as np
import scipy.sparse as snp
import scs

from stack_of_tasks.solver.HQPSolver import HqpSolver
from stack_of_tasks.tasks.Task import EqTask


class SCSSolver(HqpSolver):
    def __init__(self, number_of_joints, stack_of_tasks, **options) -> None:
        super().__init__(number_of_joints, stack_of_tasks, **options)
        self.slack_joint_matrix = None

    def _create_matrix(self):
        super()._create_matrix()
        self.slack_joint_matrix = snp.identity(self.m_slacks + self.N, format="csc")

    def _interpret_solution(self, level_index, solution: object) -> Any:
        if solution["info"]["status_val"] != 1:
            print("not solved")
            return None
        print(solution["x"])
        dq = solution["x"][: self.N]

        for task in self._stack_of_tasks.hierarchy[level_index]:
            b = task.A.dot(dq)
            if isinstance(task, EqTask):

                task.residual = b - task.bound
                task.violation = ~np.isclose(task.residual, b)
            else:
                task.residual = None
                task.violation = (task.lower_bound < b) & (b < task.upper_bound)

            self._add_nullspace(task.A, b)

        return solution["x"][: self.N]

    def _solve(self, warmstart, **options) -> Any:

        data = {
            "P": snp.csc_matrix(
                self.objective_matrix[: self.N + self.slacks, : self.N + self.slacks]
            ),
            "c": self.objective_vector[: self.N + self.slacks],
            "A": snp.vstack(
                [
                    # zero cone
                    self.nullspace_matrix[: self.ns_rows, : self.N + self.slacks],
                    self.eq_matrix[: self.eq_rows, : self.N + self.slacks],
                    # linear cone
                    self.sieq_matrix[: self.sieq_rows, : self.N + self.slacks],
                    # box cone
                    snp.csc_matrix((1, self.N + self.slacks)),
                    -self.slack_joint_matrix[: self.N + self.slacks, : self.N + self.slacks],
                    -self.ieq_matrix[: self.ieq_rows, : self.N + self.slacks],
                ],
                format="csc",
            ),
            "b": np.r_[
                self.nullspace_bound[: self.ns_rows],
                self.eq_bound[: self.eq_rows],
                self.sieq_bound[: self.sieq_rows],
                1,
                np.zeros((self.N + self.slacks + self.ieq_rows,)),
            ],
        }
        cone = {
            "z": self.ns_rows + self.eq_rows,
            "l": self.sieq_rows,
            "bl": np.r_[
                self.joints_slack_lower[: self.N + self.slacks],
                self.ieq_lower[: self.ieq_rows],
            ],
            "bu": np.r_[
                self.joints_slack_upper[: self.N + self.slacks],
                self.ieq_upper[: self.ieq_rows],
            ],
        }

        if warmstart is not None:
            data["x"] = warmstart

        s = scs.SCS(data, cone, verbose=False)
        sol = s.solve()
        return sol

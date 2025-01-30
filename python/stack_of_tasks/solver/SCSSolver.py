#!/usr/bin/env python3

from typing import Any

import numpy as np
import scipy.sparse as snp
import scs

from stack_of_tasks import syringe
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.solver.HQPSolver import HqpSolver
from stack_of_tasks.tasks import EqTask
from stack_of_tasks.tasks.hierarchy import TaskHierarchy


class SCSSolver(HqpSolver):
    @syringe.inject
    def __init__(self, robot_model: RobotModel, task_hierarchy: TaskHierarchy = None, **options) -> None:
        super().__init__(robot_model, task_hierarchy, **options)
        self.slack_joint_matrix = None

    def _create_matrix(self):
        super()._create_matrix()
        self.slack_joint_matrix = snp.identity(self.m_slacks + self.N, format="csc")

    def _interpret_solution(self, level_index, solution: object) -> Any:
        if solution["info"]["status_val"] != 1:
            print(solution["info"])
            return None
        dq = solution["x"][: self.N]

        for task in self._task_hierarchy.levels[level_index]:
            b = task.A.dot(dq)
            self._add_nullspace(task.A, b)

        return solution["x"][: self.N]

    def _solve(self, warmstart, **options) -> Any:
        data = {
            "P": snp.csc_matrix(self.objective_matrix[: self.N + self.slacks, : self.N + self.slacks]),
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

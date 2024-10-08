from typing import Any, TypedDict

import numpy as np
import osqp
import scipy.sparse as snp
from scipy.linalg import block_diag

from stack_of_tasks import syringe
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.tasks import EqTask, IeqTask, TaskSoftnessType
from stack_of_tasks.tasks.hierarchy import TaskHierarchy
from stack_of_tasks.utils import prettyprintMatrix

pp = lambda x: print(prettyprintMatrix(x))


class OSQPSolver(Solver):
    """This variant of the SoT-Solver enables the usage of variable sized tasks by creatiung new matrices per iteration."""

    @syringe.inject
    def __init__(self, robot_model: RobotModel, task_hierarchy=None, **options) -> None:
        super().__init__(robot_model, task_hierarchy, **options)

        self.rho = options.get("rho", 0.1)

    def _create_problem_matrices(self, level: int, l, u, solved_A, solved_b):
        As = []
        ls = []
        us = []

        Aslacks = []

        for task in self._task_hierarchy.levels[level]:

            task_A = task.A  # needed to get any recomputing

            if task.task_size == 0:
                continue

            As.append(task_A)

            if isinstance(task, EqTask):

                us.append(task.bound)
                ls.append(task.bound)
            elif isinstance(task, IeqTask):

                us.append(task.upper_bound)
                ls.append(task.lower_bound)

            else:
                pass

            if task.softness_type == TaskSoftnessType.linear:
                Aslacks.append(np.ones((task.task_size, 1)))
            elif task.softness_type == TaskSoftnessType.quadratic:
                Aslacks.append(np.identity(task.task_size))
            else:
                Aslacks.append(np.ndarray((task.task_size, 0)))

        if solved_A is not None and solved_b is not None:
            As.append(solved_A)
            ls.append(solved_b)
            us.append(solved_b)

            Aslacks.append(np.ndarray((solved_b.shape[0], 0)))

        if len(As) == 0:
            return None, None, None, None, None

        Atask = np.vstack(As)
        Atask_slack = block_diag(*Aslacks)

        task_lower = np.concatenate(ls)
        task_upper = np.concatenate(us)

        nvars, nslacks = Atask.shape[1], Atask_slack.shape[1]

        A = snp.vstack(
            [np.identity(nvars + nslacks), np.hstack([Atask, Atask_slack])], format="csc"
        )

        P = np.identity(nvars + nslacks)
        P[:nvars] *= self.rho
        P = snp.csc_matrix(P)

        q = np.zeros((nvars + nslacks,))

        lower = np.concatenate(
            [l, np.full((nslacks,), np.NINF), task_lower],
        )
        upper = np.concatenate([u, np.full((nslacks,), np.PINF), task_upper])

        assert A.shape[0] == lower.shape[0]
        return P, q, A, lower, upper

    def tasks_changed(self):
        pass

    def _create_solved_matrices(self, level, dq, sA_old, sb_old):

        sA = [] if sA_old is None else [sA_old]
        sb = [] if sb_old is None else [sb_old]

        for task in self._task_hierarchy[level]:
            b = task.A.dot(dq.T)
            sA.append(task.A)
            sb.append(b)

        if len(sA) == 0:
            return None, None

        return np.vstack(sA), np.concatenate(sb)

    def solve(self, lower_dq, upper_dq, warmstart, **options):

        sA = None
        sb = None

        for l in range(len(self._task_hierarchy)):
            solver = osqp.OSQP()
            P, q, A, lower, upper = self._create_problem_matrices(
                l, lower_dq, upper_dq, sA, sb
            )
            if P is None:
                continue

            solver.setup(P, q, A, lower, upper, verbose=False)

            if warmstart is not None:
                warmstart = np.concatenate([warmstart, np.zeros(len(q) - self.N)])
                solver.warm_start(warmstart)

            solution = solver.solve()

            dq = solution.x[: self.N]
            sA, sb = self._create_solved_matrices(l, dq, sA, sb)

        if any([x is None for x in dq]):
            return None

        return dq

#!/usr/bin/env python

import numpy as np
import osqp
from scipy import sparse

from .AbstactSolver import Solver

# from stack_of_tasks.utils import prettyprintMatrix


class HQPwithSlacks:
    """The hierarchical solver iteratively solves the tasks at the various levels.
    At a given level, the task is specified by lb - slack ≤ J dq ≤ ub + slack.
    Additionally, we need to fulfill the constraints of the previous tasks: J' dq = slack'.

    Thus, we can use fixed size constraint and optimization matrices A and H, which are incrementally filled.
    The maximal number of slack variables for optimization is the number of soft DoFs of a task at a given level.
    The constraint matrix has the following shape:
                         ⎧1
                      dq ⎨ ⋱
                         ⎩   1
                         ⎧     1
     max slack variables ⎨      ⋱
                         ⎩        0
     higher-level tasks  {   J'   0
     cur. task ub        {   J   -1
     cur. task lb        {   J   +1
    """

    @staticmethod
    def get_matrix_specs(task_stack):
        max_rows = 0
        row_sum = 0
        max_slacks = 0

        for tasks_same_level in task_stack:
            level_rows = 0
            for t in tasks_same_level:
                level_rows += t.size
            max_slacks = max(max_slacks, level_rows)
            max_rows = max(max_rows, row_sum + 2 * level_rows)
            row_sum += level_rows

        return max_rows, max_slacks

    def __init__(self, N, rho, tasks, lower, upper) -> None:
        super().__init__()

        max_rows, self.max_slacks = self.get_matrix_specs(tasks)
        self.tasks = tasks

        self.N = N  # number of joints
        self.max_vars = self.N + self.max_slacks  # variable order: dq, slack

        # (quadratic) objective matrix (Hessian): minimize rho*|dq|² + |slack|²
        self.H = np.identity(self.max_vars)
        self.H[:N, :N] *= rho
        # self.H[N:, N:] *= 50

        self.q = np.zeros(self.max_vars)  # (linear) objective vector

        self.A = np.zeros(
            (self.max_vars + max_rows, self.max_vars)
        )  # constraint matrix for all tasks
        self.lb = np.zeros(self.max_vars + max_rows)  # lower bound vector for all tasks
        self.ub = np.zeros(self.max_vars + max_rows)  # upper         "

        self.A[: self.N, : self.N] = np.identity(
            self.N
        )  # joint constraint matrix is identity
        self.lb[: self.N] = lower  # lower joint bounds
        self.lb[self.N : self.max_vars] = 0  # lower slack bounds

        self.ub[: self.N] = upper  # upper joint bounds
        self.ub[self.N : self.max_vars] = np.PINF  # upper slack bounds

    def __iter__(self):  # initialize iterator
        self._index = -1
        self.start_row = self.max_vars
        return self

    def __next__(self):  # forward iterator: return QP problem of next hierarchy level
        self._index += 1
        if self._index == len(self.tasks):
            raise StopIteration

        # configure upper bound constraints: -∞ ≤ J dq - slack ≤ ub
        used_vars = self.N  # number of optimization variables: dq and slacks
        used_rows = self.start_row
        for task in self.tasks[self._index]:
            rows = task.size
            self.A[used_rows : used_rows + rows, : self.N] = task.A
            self.A[used_rows : used_rows + rows, used_vars : used_vars + rows] = -np.eye(rows)
            self.lb[used_rows : used_rows + rows] = np.NINF
            self.ub[used_rows : used_rows + rows] = task.upper

            used_vars += rows
            used_rows += rows

        # configure lower bound constraints: lb ≤ J dq + slack ≤ +∞
        used_vars = self.N
        for task in self.tasks[self._index]:
            rows = task.size
            self.A[used_rows : used_rows + rows, : self.N] = task.A
            self.A[used_rows : used_rows + rows, used_vars : used_vars + rows] = np.eye(rows)
            self.lb[used_rows : used_rows + rows] = task.lower
            self.ub[used_rows : used_rows + rows] = np.PINF

            used_vars += rows
            used_rows += rows

        self.A[self.N : used_vars, self.N : used_vars] = np.eye(
            used_vars - self.N
        )  # slack constraints
        unused = self.max_vars - used_vars  # unused slack variables
        self.A[used_vars : self.max_vars, used_vars : self.max_vars] = np.zeros(
            (unused, unused)
        )

        return (
            self.H[:used_vars, :used_vars],
            self.q[:used_vars],
            self.A[:used_rows, :used_vars],
            self.lb[:used_rows],
            self.ub[:used_rows],
        )

    def nullspace_constraint(self, dq):
        # Replace upper-bound part of the current task-level's constraints with nullspace constraint
        # J' dq = fixed, i.e. don't become worse than current, higher-priority solution
        used_rows = 0
        for task in self.tasks[self._index]:
            rows = task.size
            fixed = task.A.dot(dq)
            assert np.allclose(
                self.A[self.start_row : self.start_row + rows, : self.N], task.A
            )

            self.A[
                self.start_row : self.start_row + rows, self.N :
            ] = 0  # clear slacks from constraint rows
            self.lb[self.start_row : self.start_row + rows] = fixed
            self.ub[self.start_row : self.start_row + rows] = fixed

            used_rows += rows
            self.start_row += rows  # increase start position for next iteration

        self.A[
            self.start_row : self.start_row + used_rows, self.N :
        ] = 0  # clear slacks from all remaining (lb) rows


class HQPSolver(Solver):
    def __init__(self, number_of_joints, **options) -> None:
        super().__init__(number_of_joints, **options)
        self.rho = options.get("rho")

    def solve(self, stack_of_tasks, lower_dq, upper_dq, **options):
        dq = options.get("warmstart")
        task_residuals = []

        desc = HQPwithSlacks(self.N, self.rho, stack_of_tasks, lower_dq, upper_dq)
        for qp in desc:  # iterate over QPs corresponding to hierarchy levels
            sol = self._solve_qp(*qp, dq)

            if sol.info.status_val < 0:
                print(sol.info.status)
                # return None, None      # TODO handling of failed task!
            else:
                dq, slack = sol.x[: self.N], sol.x[self.N :]
                desc.nullspace_constraint(dq)
                task_residuals.append(slack)

        return dq, task_residuals

    def _solve_qp(self, H, q, A, lb, ub, warmstart=None):
        H = sparse.csc_matrix(H)
        A = sparse.csc_matrix(A)

        m = osqp.OSQP()
        m.setup(P=H, q=q, A=A, l=lb, u=ub, verbose=False)
        if warmstart is not None:
            m.warm_start(x=np.r_[warmstart, np.full(q.size - warmstart.size, 0)])

        sol = m.solve()
        return sol

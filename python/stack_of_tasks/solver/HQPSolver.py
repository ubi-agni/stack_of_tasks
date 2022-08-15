#!/usr/bin/env python

import numpy as np
import osqp
from scipy import sparse

from .AbstactSolver import Solver
from ..tasks.Task import EQTaskDesc


class HQPComponentSlacks:
    """
    Iterator to form QP problems for the stack of tasks.
    This variant considers soft constraints via componentwise slack variables,
    i.e. by lb - slack ≤ J dq ≤ ub + slack.
    For large errors, this results in strong deviations from straight-line solution.

    The constraints of all previous tasks are considered via equality constraints:
    J' dq = J' dq'.

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

    def __init__(self, N, tasks, lower, upper, rho) -> None:
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

    def nullspace_constraint(self, x):
        # Replace upper-bound part of the current task-level's constraints with nullspace constraint
        # J' dq = fixed, i.e. don't become worse than current, higher-priority solution
        dq = x[: self.N]
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

        return dq, x[self.N :]


class HQPLinearSlacks:
    """
    Iterator to form QP problems for the stack of tasks.
    This variant considers soft equality constraints via linear scale variables λ,
    i.e. J dq = λ * ξ, maximizing λ subject to 0 ≤ λ ≤ 1.

    As we need to formulate a minimization problem, we actually minimize s = 1-λ.

    The constraints of all previous tasks are considered via equality constraints:
    J' dq = J' dq'.

    Thus, we can use fixed size constraint and optimization matrices A and H, which are incrementally filled.
    The maximal number of slack variables for optimization is the number of soft DoFs of a task at a given level.
    The constraint matrix has the following structure:
                               A           lb   ub
                         ⎧1                 0    1
     max slack variables ⎨ ⋱
                         ⎩   1              0    1
                         ⎧     1           lb   ub
                      dq ⎨      ⋱
                         ⎩        1        lb   ub
     higher-level tasks  {  0   J'          J' dq'
     hard eq. task       {  0   J           ξ    ξ                    J dq = ξ
     soft eq. task       {  ξ   J           ξ    ξ              s ξ + J dq = ξ
     hard ineq. task     {  0   J          lb   ub         lb ≤       J dq ≤ ub
    (soft ineq. task     {  1   J          lb   ub         lb ≤ 1 s + J dq ≤ ub)
    """

    def __init__(self, N, tasks, lower, upper, rho) -> None:
        super().__init__()
        self.tasks = tasks
        self.max_slacks = np.amax(
            [np.sum([not task.is_hard for task in level]) for level in tasks]
        )
        max_constraints = np.sum([task.size for level in tasks for task in level])

        self.N = N  # number of joints
        self.max_vars = self.N + self.max_slacks  # variable order: slack, dq

        # objective to minimize: rho*|dq|² + |slack|²
        self.H = np.identity(self.max_vars)  # Hessian matrix
        np.fill_diagonal(self.H[-N:, -N:], rho)
        self.q = np.zeros(self.max_vars)  # gradient vector

        # allocate constraint matrix and lb/ub vectors once
        self.A = np.zeros((self.max_vars + max_constraints, self.max_vars))
        self.lb = np.zeros(self.max_vars + max_constraints)
        self.ub = np.zeros(self.max_vars + max_constraints)

        # (fixed) direct variable constraints (slack and dq)
        np.fill_diagonal(self.A[: self.max_vars, : self.max_vars], 1.0)
        self.lb[self.max_slacks : self.max_vars] = lower  # lower joint bounds
        self.ub[self.max_slacks : self.max_vars] = upper  # upper joint bounds

        self.lb[: self.max_slacks] = 0.0  # lower slack bounds
        self.ub[: self.max_slacks] = 1.0  # upper slack bounds

    def __iter__(self):  # initialize iterator
        self._index = -1
        self.start_row = self.max_vars
        return self

    def __next__(self):  # forward iterator: return QP problem of next hierarchy level
        self._index += 1
        if self._index == len(self.tasks):
            raise StopIteration

        # configure upper bound constraints: -∞ ≤ J dq - slack ≤ ub
        used_slacks = 0
        used_rows = self.start_row
        for task in self.tasks[self._index]:
            rows = slice(used_rows, used_rows + task.size)
            self.A[rows, : self.N] = 0.0
            self.A[rows, -self.N :] = task.A
            if isinstance(task, EQTaskDesc):
                if task.is_hard:
                    self.lb[rows] = self.ub[rows] = task.upper
                else:
                    used_slacks += 1
                    self.A[rows, self.max_slacks - used_slacks] = task.upper
                    self.lb[rows] = self.ub[rows] = task.upper
            else:  # inequality task
                self.lb[rows] = task.lower
                self.ub[rows] = task.upper

            used_rows += task.size

        vars = slice(self.max_slacks - used_slacks, None)
        rows = slice(self.max_slacks - used_slacks, used_rows)
        return (
            self.H[vars, vars],
            self.q[vars],
            self.A[rows, vars],
            self.lb[rows],
            self.ub[rows],
        )

    def nullspace_constraint(self, x):
        # Replace current task-level's constraints with nullspace constraint
        # J' dq = fixed, i.e. don't become worse than current, higher-priority solution
        level = self.tasks[self._index]
        used_rows = np.sum([task.size for task in level])
        used_slacks = np.sum([task.is_hard for task in level])
        dq = x[-self.N :]
        rows = slice(self.start_row, self.start_row + used_rows)
        self.lb[rows] = self.ub[rows] = self.A[rows, -self.N :].dot(dq)  # fix task delta
        self.A[rows, : self.max_slacks] = 0  # zero slack contribution

        return dq, x[self.max_slacks - used_slacks : self.max_slacks]


class HQPSolver(Solver):
    def __init__(self, number_of_joints, solver=None, **options) -> None:
        super().__init__(number_of_joints)
        self.options = options
        if solver == "components":
            self.HQP = HQPComponentSlacks
        else:
            self.HQP = HQPLinearSlacks

    def solve(self, tasks, lower_dq, upper_dq, **options):
        dq = options.get("warmstart")
        task_residuals = []

        desc = self.HQP(self.N, tasks, lower_dq, upper_dq, **self.options)
        for qp in desc:  # iterate over QPs corresponding to hierarchy levels
            # print("Level: {level}".format(level=desc._index))
            # print(np.vstack(qp[0])) # H
            # print(np.hstack([qp[2], qp[3][:,np.newaxis], qp[4][:,np.newaxis]])) # A, lb, ub
            sol = self._solve_qp(*qp, dq)

            if sol.info.status_val < 0:
                print(sol.info.status)
                # return None, None      # TODO handling of failed task!
            else:
                dq, slack = desc.nullspace_constraint(sol.x)
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

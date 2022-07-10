#!/usr/bin/env python

import numpy as np
import osqp
from scipy import sparse


def prettyprintMatrix(inmatrix):
    matrix = []
    maxL = 0
    for row in inmatrix:
        col = []
        for x in row:
            s = f"{x:.2e}"
            maxL = max(len(s), maxL)
            col.append(s)
        matrix.append(col)

    s = ""
    for r in matrix:

        for x in r:
            s += f" {x :>^{maxL}} "
        s += "\n"
    print(s)


class QPDescWithFixedSize:
    @staticmethod
    def get_matrix_specs(task_stack):
        max_cols = 0
        col_sum = 0
        max_num_slack = 0

        for tasks_same_level in task_stack:
            level_cols = 0
            for t in tasks_same_level:
                level_cols += t.size
            max_num_slack = max(max_num_slack, level_cols)
            max_cols = max(max_cols, col_sum + 2 * level_cols)
            col_sum += level_cols

        return max_cols, max_num_slack

    def __init__(self, N, rho, sot, lower, upper) -> None:
        super().__init__()

        number_of_columns, self.number_of_slack = self.get_matrix_specs(sot)
        self.l = sot

        self.N = N  # number of joints
        self.numVars = self.N + self.number_of_slack

        self.H = np.identity(self.numVars)  # objective matrix
        self.H[:N, :N] *= rho

        self.q = np.zeros(self.numVars)  # objective vector

        self.A = np.zeros(
            (self.numVars + number_of_columns, self.numVars)
        )  # constraint matrix for all tasks
        self.lb = np.zeros(
            self.numVars + number_of_columns
        )  # lower bound vector for all tasks
        self.ub = np.zeros(self.numVars + number_of_columns)  # upper         "

        self.A[: self.numVars, : self.numVars] = np.identity(
            self.numVars
        )  #   joint constraint matrix is identity
        self.lb[: self.N] = lower  # lower joint bound
        self.lb[self.N : self.numVars] = 0  # lower slack bound

        self.ub[: self.N] = upper  # upper joint bound
        self.ub[self.N : self.numVars] = np.PINF  # upper slack bound

    def __iter__(self):  # use solver description as iterator
        self._index = -1
        self.start_row = self.numVars

        return self

    def __next__(self):  # use solver description as iterator
        if self._index == len(self.l) - 1:
            raise StopIteration
        self._index += 1

        numRows = 0

        self.A[:, self.N :] = 0  # remove slacking from all tasks before

        for task in self.l[self._index]:  # first iteration for upper bound constraints
            task_size = task.size

            self.A[
                self.start_row + numRows : self.start_row + numRows + task_size,
                : self.N,
            ] = task.A
            self.lb[
                self.start_row + numRows : self.start_row + numRows + task_size
            ] = np.NINF
            self.ub[
                self.start_row + numRows : self.start_row + numRows + task_size
            ] = task.upper

            numRows += task_size
        self.A[self.start_row : self.start_row + numRows, self.N :] = -np.eye(
            numRows, self.number_of_slack
        )  # set -1 for slacks

        s = self.start_row + numRows
        numRows = 0

        for task in self.l[
            self._index
        ]:  # second iteration for inverted lower bound constraints
            task_size = task.size

            self.A[s + numRows : s + numRows + task_size, : self.N] = -task.A
            self.lb[s + numRows : s + numRows + task_size] = np.NINF
            self.ub[s + numRows : s + numRows + task_size] = -task.lower

            numRows += task_size

        self.A[
            self.start_row + numRows : self.start_row + 2 * numRows, self.N :
        ] = -np.eye(
            numRows, self.number_of_slack
        )  # set -1 for slacks

        self.H[self.N :, self.N :] = np.zeros(
            (self.number_of_slack, self.number_of_slack)
        )  # set H matrix correctly
        self.H[self.N : self.N + numRows, self.N : self.N + numRows] = np.identity(
            numRows
        )

        return (
            self.H,
            self.q,
            self.A[: self.start_row + 2 * numRows, :],
            self.lb[: self.start_row + 2 * numRows],
            self.ub[: self.start_row + 2 * numRows],
        )

    def add_slack(self, slack_vector):
        # add slack variables to upper part of the constraints and add joint upper and lower bounds in one constraint
        numRows = 0
        for task in self.l[self._index]:
            slackSize = task.size

            self.lb[self.start_row + numRows : self.start_row + numRows + slackSize] = (
                task.lower - slack_vector[numRows : numRows + slackSize]
            )
            self.ub[
                self.start_row + numRows : self.start_row + numRows + slackSize
            ] += slack_vector[numRows : numRows + slackSize]

            violations = self.ub < self.lb  # lower bound is larger than upper, happens very close to equality
            self.ub[violations], self.lb[violations] = self.lb[violations], self.ub[violations]

            numRows += slackSize

        self.start_row += numRows  # increase start position for next iteration


class Solver:
    def __init__(self, N, rho) -> None:
        self.N = N
        self.rho = rho

    def solve_sot(self, list_of_tasks, lower, upper, warmstart=None):
        task_chain_objectivs = []
        dq = warmstart

        desc = QPDescWithFixedSize(self.N, self.rho, list_of_tasks, lower, upper)

        for d in desc:
            sol = self._solve_qp(*d, dq)

            if sol.info.status_val < 0:
                print(sol.info.status)
                # return None, None      # TODO handling of failed task!
            else:
                dq, slack = sol.x[: self.N], sol.x[self.N :]

                desc.add_slack(slack)

        return dq, task_chain_objectivs

    def _solve_qp(self, H, q, A, lb, ub, warmstart=None):
        H = sparse.csc_matrix(H)
        A = sparse.csc_matrix(A)

        m = osqp.OSQP()
        m.setup(P=H, q=q, A=A, l=lb, u=ub, verbose=False)
        if warmstart is not None:
            m.warm_start(x=np.r_[warmstart, np.full(q.size - warmstart.size, 0)])

        sol = m.solve()
        return sol

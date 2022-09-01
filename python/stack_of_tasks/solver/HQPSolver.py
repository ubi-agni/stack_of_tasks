#!/usr/bin/env python


import numpy as np
import osqp
from numpy.typing import NDArray
from scipy import sparse

from stack_of_tasks.tasks.Task import (
    EqTask,
    TaskHierachyType,
    TaskSoftnessType,
    TaskType,
)

from .AbstactSolver import Solver


class HQPMatrix:
    """
                              A           lb   ub
                        ⎧1                 0    1   for lin. task
    max slack variables ⎨ ⋱             -inf   inf for quad. task
                        ⎩   0              0    1
                        ⎧    1            lb   ub
                     dq ⎨      ⋱
                        ⎩        1        lb   ub
    higher-level tasks  {  0   J'      J' dq'  J' dq'
    hard eq. task       {  0   J           ξ    ξ                    J dq = ξ
    soft eq. task       {  ξ   J           ξ    ξ              s ξ + J dq = ξ
    hard ineq. task     {  0   J          lb   ub         lb ≤       J dq ≤ ub
    quad task ub        { +1   J          lb   inf        lb ≤   s + J dq
    quad task lb        { -1   J        -inf   ubs              -s + J dq ≤ ub
    (soft ineq. task    {  1   J          lb   ub         lb ≤ 1 s + J dq ≤ ub)"""

    def __init__(self, number_of_vars, rho=0.01) -> None:
        self.sot: TaskHierachyType = []

        self.N = number_of_vars  # number of problem variables
        self.rho = rho

        # [m]aximal number of slack variables needed
        self.m_slack = 0
        # [m]aximal number of rows occupied by tasks without slack variables
        self.m_hard_r = 0
        # [m]aximal number of rows occupied by tasks with slack variables in one level
        self.m_wslack_r = 0
        # [m]aximal number of rows occupied by nullspace
        #   constraints of tasks in previous levels
        self.m_nspace_r = 0

        # starting rows for hard, slack and nullspace in A
        self._hs = self.N + self.m_slack
        self._ss = self._hs + self.m_hard_r
        self._ns = self._ss + self.m_wslack_r

        # [u]sed rows by hard tasks
        self.u_hard_r = 0
        # [u]sed rows by tasks with slack
        self.u_wslack_r = 0
        # [u]sed rows by nullspace constraint
        self.u_nspace_r = 0

        self.J = np.identity(self.N)  # J matrix
        self.p = np.zeros((self.N))  # p vector

        # internal A matrix, lower and upper bound with space for all constraints
        self._A = np.zeros((self.N, self.N))
        self._lower = np.zeros((self.N,))
        self._upper = np.zeros((self.N,))

        # stubs for r/w views on _A, _lower and _upper
        self.slack_diag: NDArray  # diagonal entries of slacks in A

        self._A_hard: NDArray  # hard tasks in A
        self._A_nullspace: NDArray  # nullspace tasks in A
        self._A_wslack: NDArray  # tasks with slack in A
        self._A_slacks: NDArray  # slack part of tasks with slacks
        # same as above but for lower and upper bounds
        self._l_dq: NDArray
        self._u_dq: NDArray
        self._l_slacks: NDArray
        self._u_slacks: NDArray
        self._l_hard: NDArray
        self._u_hard: NDArray
        self._l_slacked: NDArray
        self._u_slacked: NDArray
        self._l_nullspace: NDArray
        self._u_nullspace: NDArray

        # iterator index
        self._level_index = 0

    def set_tasks(self, sot: TaskHierachyType):
        recalc = self._check_stack_different(sot)
        self.sot = sot
        if recalc:
            self._recalculate_dimensions()

    def _recalculate_dimensions(self):
        self._calc_matrix_size()
        self._set_matrices()
        self._define_views()

    def _check_stack_different(self, new_stack: TaskHierachyType):
        if len(new_stack) == len(self.sot):
            for nl, ol in zip(new_stack, self.sot):
                if len(nl) == len(ol):
                    for tn, to in zip(nl, ol):
                        if not (type(tn) is type(to) and tn.softnessType is to.softnessType):
                            return True
            return False
        return True

    def _set_matrices(self):
        self.J = np.identity(self.N + self.m_slack)
        self.J[self.m_slack :, self.m_slack :] = self.rho
        self.p = np.zeros((self.N + self.m_slack,))

        self._A = np.zeros(
            (
                self.m_slack + self.N + self.m_hard_r + self.m_wslack_r + self.m_nspace_r,
                self.m_slack + self.N,
            )
        )

        self._lower = np.zeros(
            (self.m_slack + self.N + self.m_hard_r + self.m_wslack_r + self.m_nspace_r,)
        )

        self._upper = np.zeros(
            (self.m_slack + self.N + self.m_hard_r + self.m_wslack_r + self.m_nspace_r,)
        )

        # set diagonal entries of A for dq to 1
        self._A[
            self.m_slack : self.m_slack + self.N, self.m_slack : self.m_slack + self.N
        ] = np.identity(self.N)

    def _define_views(self):

        self.slack_diag = np.lib.stride_tricks.as_strided(
            self._A,
            shape=(self.m_slack,),
            strides=(sum(self._A.strides),),
        )

        self._A_hard = self._A[self._hs : self._hs + self.m_hard_r, self.m_slack :]
        self._A_nullspace = self._A[self._ns : self._ns + self.m_nspace_r, self.m_slack :]
        self._A_wslack = self._A[self._ss : self._ss + self.m_wslack_r, self.m_slack :]
        self._A_slacks = self._A[self._ss : self._ss + self.m_wslack_r, : self.m_slack]

        self._l_dq = self._lower[self.m_slack : self.m_slack + self.N]
        self._u_dq = self._upper[self.m_slack : self.m_slack + self.N]

        self._l_slacks = self._lower[
            : self.m_slack,
        ]

        self._u_slacks = self._upper[
            : self.m_slack,
        ]

        self._l_hard = self._lower[
            self._hs : self._hs + self.m_hard_r,
        ]

        self._u_hard = self._upper[
            self._hs : self._hs + self.m_hard_r,
        ]

        self._l_slacked = self._lower[
            self._ss : self._ss + self.m_wslack_r,
        ]

        self._u_slacked = self._upper[
            self._ss : self._ss + self.m_wslack_r,
        ]
        self._l_nullspace = self._lower[
            self._ns : self._ns + self.m_nspace_r,
        ]

        self._u_nullspace = self._upper[
            self._ns : self._ns + self.m_nspace_r,
        ]

    def _calc_matrix_size(self):
        for level in self.sot:
            slacks = 0
            rows = 0
            for task in level:
                if task.softnessType is TaskSoftnessType.hard:
                    self.m_hard_r += 1
                elif task.softnessType is TaskSoftnessType.linear:
                    slacks += 1
                    rows += task.task_size
                    self.m_nspace_r += task.task_size
                elif task.softnessType is TaskSoftnessType.quadratic:
                    slacks += task.task_size
                    rows += task.task_size * 2
                    self.m_nspace_r += task.task_size
            self.m_wslack_r = max(self.m_wslack_r, rows)
            self.m_slack = max(self.m_slack, slacks)
        self._hs = self.N + self.m_slack
        self._ss = self._hs + self.m_hard_r
        self._ns = self._ss + self.m_wslack_r

    def reset(self):
        if self.m_hard_r > 0:
            self._A_hard[:] = 0
        if self.m_nspace_r > 0:
            self._A_nullspace[:] = 0
        if self.m_wslack_r > 0:
            self._A_wslack[:] = 0
        if self.m_slack > 0:
            self._A_slacks[:] = 0
            self.slack_diag[:] = 0

        self.u_hard_r = 0
        self.u_wslack_r = 0
        self.u_nspace_r = 0

        self._level_index = 0

        return self

    def set_dq_bounds(self, lower, upper):
        self._l_dq[:] = lower
        self._u_dq[:] = upper

    def __next__(self):
        if self._level_index >= len(self.sot):
            return False

        self._A_wslack[:] = 0
        self._A_slacks[:] = 0
        self.slack_diag[:] = 0
        self._l_slacks[:] = 0
        self._u_slacks[:] = 0

        self.u_wslack_r = 0

        used_slacks = 0
        for task in self.sot[self._level_index]:
            task: TaskType
            if task.softnessType is TaskSoftnessType.hard:
                self._A_hard[self.u_hard_r : self.u_hard_r + task.task_size, :] = task.A
                (
                    self._l_hard[self.u_hard_r : self.u_hard_r + task.task_size],
                    self._u_hard[self.u_hard_r : self.u_hard_r + task.task_size],
                ) = (
                    (task.bound, task.bound)
                    if isinstance(task, EqTask)
                    else (task.lower_bound, task.upper_bound)
                )
                self.u_hard_r += task.task_size
            elif task.softnessType is TaskSoftnessType.linear and isinstance(task, EqTask):
                s = self.u_wslack_r
                e = s + task.task_size

                self.slack_diag[used_slacks] = 1
                self._l_slacks[used_slacks] = 0
                self._u_slacks[used_slacks] = 1

                self._A_wslack[s:e, :] = task.A
                self._A_slacks[s:e, used_slacks] = task.bound
                self._l_slacked[s:e] = task.bound
                self._u_slacked[s:e] = task.bound

                used_slacks += 1
                self.u_wslack_r += task.task_size

            elif task.softnessType is TaskSoftnessType.quadratic:
                s = self.u_wslack_r
                e = s + task.task_size

                self.slack_diag[used_slacks : used_slacks + task.task_size] = 1
                self._l_slacks[used_slacks : used_slacks + task.task_size] = np.NINF
                self._u_slacks[used_slacks : used_slacks + task.task_size] = np.PINF

                self._A_wslack[s:e, :] = task.A
                self._l_slacked[s:e] = (
                    task.bound if isinstance(task, EqTask) else task.lower_bound
                )
                self._u_slacked[s:e] = np.PINF
                self._A_slacks[s:e][used_slacks : used_slacks + task.task_size] = np.identity(
                    task.task_size
                )

                s += task.task_size
                e += task.task_size

                self._A_wslack[s:e, :] = task.A
                self._A_slacks[s:e][
                    used_slacks : used_slacks + task.task_size
                ] = -np.identity(task.task_size)
                self._l_slacked[s:e] = np.NINF
                self._u_slacked[s:e] = (
                    task.bound if isinstance(task, EqTask) else task.upper_bound
                )

                used_slacks += task.task_size
                self.u_wslack_r += task.task_size * 2

        self._level_index += 1
        return True

    def add_nullspace_constraint(self, solution_vector):
        dq, slacks = solution_vector[self.m_slack :], solution_vector[: self.m_slack]

        for task in self.sot[self._level_index - 1]:
            s = self.u_nspace_r
            e = s + task.task_size
            b = task.A.dot(dq)

            self._A_nullspace[s:e] = task.A
            self._l_nullspace[s:e] = b
            self._u_nullspace[s:e] = b

            self.u_nspace_r += task.task_size

        return dq, slacks

    @property
    def A(self):
        return self._A[
            np.r_[
                0 : self.N + self.m_slack,
                self._hs : self._hs + self.u_hard_r,
                self._ss : self._ss + self.u_wslack_r,
                self._ns : self._ns + self.u_nspace_r,
            ],
            :,
        ]

    @property
    def lower(self):
        return self._lower[
            np.r_[
                : self.N + self.m_slack,
                self._hs : self._hs + self.u_hard_r,
                self._ss : self._ss + self.u_wslack_r,
                self._ns : self._ns + self.u_nspace_r,
            ],
        ]

    @property
    def upper(self):
        return self._upper[
            np.r_[
                : self.N + self.m_slack,
                self._hs : self._hs + self.u_hard_r,
                self._ss : self._ss + self.u_wslack_r,
                self._ns : self._ns + self.u_nspace_r,
            ],
        ]


class HQPSolver(Solver):
    def __init__(self, number_of_joints, **options) -> None:
        super().__init__(number_of_joints)
        self.options = options

        self.hqp_mats = HQPMatrix(self.N, **options)

    def solve(self, stack_of_tasks, lower_dq, upper_dq, **options):
        dq = options.get("warmstart")
        task_residuals = []

        self.hqp_mats.reset()
        self.hqp_mats.set_tasks(stack_of_tasks)
        self.hqp_mats.set_dq_bounds(lower_dq, upper_dq)

        while next(self.hqp_mats):  # iterate over QPs corresponding to hierarchy levels
            sol = self._solve_qp(
                self.hqp_mats.J,
                self.hqp_mats.p,
                self.hqp_mats.A,
                self.hqp_mats.lower,
                self.hqp_mats.upper,
                dq,
            )

            if sol.info.status_val < 0:
                print(sol.info.status)
                # return None, None      # TODO handling of failed task!
            else:

                dq, slack = self.hqp_mats.add_nullspace_constraint(sol.x)
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

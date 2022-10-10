#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import Any

import numpy as np
from numpy.typing import NDArray

from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.tasks.Task import EqTask, TaskSoftnessType, TaskTypes
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy


class HqpSolver(Solver):

    r"""The HQPSolver Base class.
                             P
                        ⎧rho
    dq                  ⎨    ⋱
                        ⎩       rho
                        ⎧           1
    slacks              ⎨             ⋱
                        ⎩               1

                             A                        lb/b      ub         formula
    joint constraint         I                       jl         ju
    slack constraint                     I               depending on task type

                             ns_matrix                ns_bound
    higher-level tasks  {    J'      0 ⋯ 0 ⋯ 0        J' dq'     ∅

                             eq_matrix                eq_bound
    hard eq. task       {    J       0 ⋯ 0 ⋯ 0        ξ          ∅      ξ = J dq
    soft eq. task       {    J       0 ⋯ ξ  ⋯ 0       ξ          ∅      ξ = s ξ + J dq

                             ieq_matrix               ieq_upper  ieq_lower
    hard ineq. task     {    J       0 ⋯ 0 ⋯ 0        lb         ub    lb ≤ J dq ≤ ub

                            sieq_matrix             sieq_bound
    quad task lb        {    J       0 ⋯ I ⋯ 0       lb          ∅     lb ≤  J dq + s
    quad task ub        {   -J       0 ⋯ I ⋯ 0       -ub         ∅    -ub ≤  -J dq + s"""

    #  (soft ineq. task    {  1   J          lb   ub         lb ≤ 1 s + J dq ≤ ub)

    def __init__(
        self, number_of_joints: int, stack_of_tasks: TaskHierarchy, **options
    ) -> None:
        super().__init__(number_of_joints, stack_of_tasks, **options)

        self.m_slacks = 0
        self.m_eq = 0
        self.m_ieq = 0
        self.m_sieq = 0
        self.m_ns = 0

        self.ns_rows = 0
        self.eq_rows = 0
        self.ieq_rows = 0
        self.sieq_rows = 0
        self.slacks = 0

        self.objective_matrix: NDArray
        self.objective_vector: NDArray

        self.joints_slack_lower: NDArray
        self.joints_slack_upper: NDArray

        self.nullspace_matrix: NDArray
        self.nullspace_bound: NDArray

        self.eq_matrix: NDArray
        self.eq_bound: NDArray

        self.sieq_matrix: NDArray
        self.sieq_bound: NDArray

        self.ieq_matrix: NDArray
        self.ieq_upper: NDArray
        self.ieq_lower: NDArray

        self.sieq_matrix: NDArray
        self.sieq_bound: NDArray

        self._calculate_sot_sizes()
        self._create_matrix()

    def set_stack_of_tasks(self, stack_of_tasks: TaskHierarchy):
        self._stack_of_tasks = stack_of_tasks

    def stack_changed(self):
        super().stack_changed()
        self._calculate_sot_sizes()
        self._create_matrix()

    def _reset(self):
        self.ns_rows = 0
        self.joints_slack_lower[:] = 0
        self.joints_slack_upper[:] = 0

    def _calculate_sot_sizes(self):
        self.level_slack_mapping = []

        self.m_slacks = 0
        self.m_eq = 0
        self.m_ieq = 0
        self.m_sieq = 0
        self.m_ns = 0

        # find number of slacks
        for level in self._stack_of_tasks.hierarchy:
            level_slacks = level_ieq_r = level_sieq_r = level_eq_r = 0

            for task in level:

                if task.softnessType is TaskSoftnessType.quadratic:
                    level_slacks += task.task_size
                    level_sieq_r += 2 * task.task_size
                else:
                    if task.softnessType is TaskSoftnessType.linear:
                        level_slacks += 1

                    if isinstance(task, EqTask):
                        level_eq_r += task.task_size
                    else:
                        level_ieq_r += task.task_size

                self.m_ns += task.task_size

            self.m_slacks = max(self.m_slacks, level_slacks)
            self.m_ieq = max(self.m_ieq, level_ieq_r)
            self.m_sieq = max(self.m_sieq, level_sieq_r)
            self.m_eq = max(self.m_eq, level_eq_r)

    def _create_matrix(self):
        # Objective matrix P and vector q
        self.objective_matrix = np.identity(self.N + self.m_slacks)
        self.objective_matrix[self.N :, self.N :] = self._options.get("rho", 0.1)

        self.objective_vector = np.zeros((self.N + self.m_slacks,))

        # lower and upper bounds for joints and slacks,
        # if needed corresponding identity matrix is created in concrete solver
        self.joints_slack_lower = np.zeros((self.N + self.m_slacks,))
        self.joints_slack_upper = np.zeros((self.N + self.m_slacks,))

        # nullspace matrix and bound
        self.nullspace_matrix = np.zeros((self.m_ns, self.N + self.m_slacks))
        self.nullspace_bound = np.zeros((self.m_ns,))

        # matrix and bounds for box-inequalities
        self.ieq_matrix = np.zeros((self.m_ieq, self.N + self.m_slacks))
        self.ieq_upper = np.zeros((self.m_ieq,))
        self.ieq_lower = np.zeros((self.m_ieq,))

        # matrix and bound for single-sided inequalities
        self.sieq_matrix = np.zeros((self.m_sieq, self.N + self.m_slacks))
        self.sieq_bound = np.zeros((self.m_sieq,))

        # matrix and bound for equalities
        self.eq_matrix = np.zeros((self.m_eq, self.N + self.m_slacks))
        self.eq_bound = np.zeros((self.m_eq,))

    def _set_bounds(self, lb, ub):
        self.joints_slack_lower[: self.N] = lb
        self.joints_slack_upper[: self.N] = ub

    def _add_nullspace(self, matrix: NDArray, bound):
        rows = bound.shape[0]
        self.nullspace_matrix[self.ns_rows : self.ns_rows + rows, : self.N] = matrix
        self.nullspace_bound[self.ns_rows : self.ns_rows + rows] = bound
        self.ns_rows += rows

    def _prepare_level(self, level_index) -> Any:
        self.eq_rows = 0
        self.ieq_rows = 0
        self.sieq_rows = 0
        self.slacks = 0

        for task in self._stack_of_tasks.hierarchy[level_index]:
            if task.is_task_type(TaskTypes.HARD_EQ):

                self.eq_bound[self.eq_rows : self.eq_rows + task.task_size] = task.bound
                self.eq_matrix[
                    self.eq_rows : self.eq_rows + task.task_size, : self.N
                ] = task.A

            elif task.is_task_type(TaskTypes.HARD_IEQ):

                self.ieq_lower[
                    self.ieq_rows : self.ieq_rows + task.task_size
                ] = task.lower_bound

                self.ieq_upper[
                    self.ieq_rows : self.ieq_rows + task.task_size
                ] = task.upper_bound

                self.ieq_matrix[
                    self.ieq_rows : self.ieq_rows + task.task_size, : self.N
                ] = task.A

                self.ieq_rows += task.task_size

            elif task.is_task_type(TaskTypes.LINEAR_EQ):
                # slack variables in linear eqs have bounds [0,1]
                self.joints_slack_lower[self.N + self.slacks] = 0
                self.joints_slack_upper[self.N + self.slacks] = 1

                self.eq_bound[self.eq_rows : self.eq_rows + task.task_size] = task.bound

                self.eq_matrix[
                    self.eq_rows : self.eq_rows + task.task_size, : self.N
                ] = task.A

                self.eq_matrix[
                    self.eq_rows : self.eq_rows + task.task_size, self.N + self.slacks
                ] = task.bound

                self.slacks += 1
                self.eq_rows += task.task_size

            elif task.is_task_type(TaskTypes.QUADRATIC):
                # Quadratic (in) equalities are split into lower and upper bounds and are transferred
                # to be only lower bounds.

                self.joints_slack_lower[
                    self.N + self.slacks : self.N + self.slacks + task.task_size
                ] = -np.inf

                self.joints_slack_upper[
                    self.N + self.slacks : self.N + self.slacks + task.task_size
                ] = np.inf

                self.sieq_bound[self.sieq_rows : self.sieq_rows + task.task_size] = (
                    task.bound if isinstance(task, EqTask) else task.lower_bound
                )

                self.sieq_matrix[
                    self.sieq_rows : self.sieq_rows + task.task_size, : self.N
                ] = task.A

                self.sieq_matrix[
                    self.sieq_rows : self.sieq_rows + task.task_size, self.N :
                ] = np.eye(task.task_size, self.m_slacks, self.slacks)

                self.sieq_rows += task.task_size

                self.sieq_bound[self.sieq_rows : self.sieq_rows + task.task_size] = -(
                    task.bound if isinstance(task, EqTask) else task.upper_bound
                )

                # upper bounds are swapped
                self.sieq_matrix[
                    self.sieq_rows : self.sieq_rows + task.task_size, : self.N
                ] = -task.A

                self.sieq_matrix[
                    self.sieq_rows : self.sieq_rows + task.task_size, self.N :
                ] = np.eye(task.task_size, self.m_slacks, self.slacks)

                self.sieq_rows += task.task_size
                self.slacks += task.task_size

            else:
                raise NotImplementedError("Not Implemented Task Type")

    @abstractmethod
    def _interpret_solution(self, level_index, solution) -> Any:
        pass

    @abstractmethod
    def _solve(self, warmstart, **options):
        pass

    def solve(self, lower_dq, upper_dq, **options):
        self._reset()
        self._set_bounds(lower_dq, upper_dq)
        dq_warmstart = options.pop("warmstart")

        for level_index, _ in enumerate(self._stack_of_tasks.hierarchy):
            # prepare matrices

            self._prepare_level(level_index)
            solution = self._solve(dq_warmstart, **options)

            solution = self._interpret_solution(level_index, solution)
            if solution is not None:
                dq_warmstart = solution

        return dq_warmstart

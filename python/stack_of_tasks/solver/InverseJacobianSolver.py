from typing import List

import numpy as np

from stack_of_tasks.tasks.Task import EqTask

from .AbstractSolver import Solver


class InverseJacobianSolver(Solver):
    def __init__(
        self, number_of_joints, stack_of_tasks: List[List[EqTask]], **options
    ) -> None:
        super().__init__(number_of_joints, stack_of_tasks, **options)
        self.threshold = options.get("threshold", 0.01)

    def stack_changed(self):
        # TODO check if stack consists of only EQ-Tasks, Warn if task is not Hard (type will be ignored in this solver)
        # count task-sizes, error larger than self.N (can not be solved by this solver)
        pass

    def _invert_smooth_clip(self, s):
        return s / (self.threshold**2) if s < self.threshold else 1.0 / s

    def solve(self, lower_dq, upper_dq, **options):
        N = np.identity(self.N)  # nullspace projector of previous tasks
        JA = np.zeros((0, self.N))  # accumulated Jacobians

        qdot = np.zeros(self.N)

        residuals = []

        for task_level in self._stack_of_tasks.hierarchy:

            # combine tasks of this level into one
            J = np.concatenate([task.A for task in task_level], axis=0)
            e = np.concatenate([task.bound for task in task_level], axis=0)

            U, S, Vt = np.linalg.svd(J.dot(N))  # * self.joint_weights[None, :]

            # compute V'.T = V.T * Mq.T
            # Vt *= self.joint_weights[None, :]

            rank = min(U.shape[0], Vt.shape[1])
            for i in range(rank):
                S[i] = self._invert_smooth_clip(S[i])

            qdotn = np.dot(Vt.T[:, 0:rank], S * U.T.dot(np.array(e) - J.dot(qdot))).reshape(
                qdot.shape
            )

            qdot += qdotn
            residuals.append(np.array(e) - J.dot(qdot))

            # compute new nullspace projector
            JA = np.vstack([JA, J])
            U, S, Vt = np.linalg.svd(JA)
            accepted_singular_values = (S > 1e-3).sum()
            VN = Vt[accepted_singular_values:].T
            N = VN.dot(VN.T)

        # self.nullspace = VN  # remember nullspace basis

        # uniformly scale qdot if any limit is exceeded
        scales = np.maximum(qdot / lower_dq, qdot / upper_dq)
        scales[np.logical_not(np.isfinite(scales))] = 1.0
        qdot /= np.maximum(1.0, np.max(scales))

        for l in self._stack_of_tasks.hierarchy:
            for t in l:
                t: EqTask
                t.residual = (t.A @ qdot) - t.bound
                t.violation = np.allclose(t.residual, 0)

        return qdot

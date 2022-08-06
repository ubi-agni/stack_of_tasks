import numpy as np

from .AbstactSolver import Solver


class InverseJacobianSolver(Solver):
    def __init__(self, number_of_joints, options) -> None:
        super().__init__(number_of_joints, options)

        self.threshold = options.get("threshold", 0.01)

    def _invert_smooth_clip(self, s):
        return s / (self.threshold**2) if s < self.threshold else 1.0 / s

    def solve(self, stack_of_tasks, lower_dq, upper_dq, options):
        N = np.identity(self.N)  # nullspace projector of previous tasks
        JA = np.zeros((0, self.N))  # accumulated Jacobians
        qdot = np.zeros(self.N)

        for task_level in stack_of_tasks:

            # combine tasks of this level into one
            J = np.concatenate([task.A for task in task_level], axis=0)
            e = np.concatenate([task.upper for task in task_level], axis=0)

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

            # compute new nullspace projector
            JA = np.vstack([JA, J])
            U, S, Vt = np.linalg.svd(JA)
            accepted_singular_values = (S > 1e-3).sum()
            VN = Vt[accepted_singular_values:].T
            N = VN.dot(VN.T)

        self.nullspace = VN  # remember nullspace basis
        qdot = np.maximum(qdot, lower_dq)
        qdot = np.minimum(qdot, upper_dq)
        return qdot, None

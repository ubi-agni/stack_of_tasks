from task import TaskDesc
import numpy as np
from scipy import sparse

import osqp

class QpDesc:
    def __init__(self, N) -> None:

        self.J = np.zeros((0,N+1)) 
        self.g = np.zeros(0) 
        
        self.A = np.zeros((0,N+1))
        self.lbA = np.zeros(0)
        self.ubA = np.zeros(0)
    
    def get_P_q(self):
        shape = self.J.shape

        if shape[0] == 0:
            return np.identity(shape[1]), np.zeros(shape[1])

    def addTask(self, desc: TaskDesc, slack=None):
        At, ut, lt = desc.unpack_only_upper()

        if slack:
            ut = ut + slack
            At = np.c_[At, np.zeros(At.shape[0])]
        else:
            At = np.c_[At, -np.ones(At.shape[0])]

        self.A = np.r_[self.A, At]
        self.lbA = np.r_[self.lbA, lt]
        self.ubA = np.r_[self.ubA, ut]



class Solver:
    def __init__(self, N, lb, ub) -> None:
        self.N = N

        self.lb = lb
        self.ub = ub



    def solve_sot(self, task_descs, warmstart=None):
        assert len(task_descs)>0

        w = []
        dq = warmstart


        for i in range(1,len(task_descs)+1):
            tis = task_descs[:i]
            sol = self._solve_part(tis, w, warmstart)

            w.append(sol.x[-1])
            dq = sol.x[:-1]

        return dq


    def _solve_part(self, tasks, wis, warmstart):
        qpD = QpDesc(self.N)

        lb = np.r_[self.lb, -10]
        ub = np.r_[self.ub, 10]

        qpD.addTask(tasks[-1])
        for i, t in enumerate(tasks[:-1]):
            qpD.addTask(t, wis[i])

        P, g = qpD.get_P_q()
        
        A = qpD.A
        ubA = qpD.ubA
        lbA = qpD.lbA

        ubA = np.r_[ubA, ub]
        lbA = np.r_[lbA, lb]
        A = np.r_[A, np.identity(lb.size)]

        A = sparse.csc_matrix(A)
        P = sparse.csc_matrix(P)
        
        m = osqp.OSQP()
        m.setup(P=P, q=g, A=A, l=lbA, u=ubA, verbose=False)

        if warmstart is not None:
            m.warm_start(x=np.r_[warmstart, 0])

        sol = m.solve()
        return sol
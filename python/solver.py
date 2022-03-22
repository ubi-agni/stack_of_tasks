#!/usr/bin/env python

from task import TaskDesc
import numpy as np
from scipy import sparse

import osqp

class QpDesc:
    def __init__(self, N, rho, lb, ub) -> None:
        self.N = N
        
        self._P = np.identity(N) * rho
        self._q = np.zeros(self.N)

        self._As = np.zeros((0,N))
        self._An = np.identity(lb.size)

        self._lbAs = np.zeros(0)
        self._ubAs = np.zeros(0)

        self._lbAn = lb
        self._ubAn = ub

    @property
    def H(self):
        H = np.identity(self.N + self._As.shape[0])
        H[:self.N, :self.N] = self._P
        return H

    @property
    def q(self):
        return np.r_[self._q, np.zeros(self._As.shape[0])]

    @property
    def A(self):
        return np.block(
            [[self._As, -np.identity(self._As.shape[0])],
            [self._An, np.zeros( (self._An.shape[0], self._As.shape[0])) ]]
        )

    @property
    def lb(self):
        return np.r_[self._lbAs, self._lbAn]

    @property
    def ub(self):
        return np.r_[self._ubAs, self._ubAn]



    def addTask(self, desc: TaskDesc, slack=None):
        At, ut, lt = desc.unpack()

        if slack is not None:
            ut = ut + slack
            lt = lt + slack

            self._An = np.r_[self._An, At]  
            self._lbAn = np.r_[self._lbAn, lt]
            self._ubAn = np.r_[self._ubAn, ut]
        else:
            self._As = np.r_[self._As, At]  
            self._lbAs = np.r_[self._lbAs, lt]
            self._ubAs = np.r_[self._ubAs, ut]




class Solver:
    def __init__(self, N, lb, ub, rho) -> None:
        self.N = N

        self.lb = lb
        self.ub = ub
        self.r = rho



    def solve_sot(self, task_descs, warmstart=None):
        assert len(task_descs)>0

        w = []
        task_chain_objectivs = []
        dq = warmstart


        for i in range(1,len(task_descs)+1):
            tis = task_descs[:i]
            sol = self._solve_part(tis, w, warmstart)
            
            
            if sol.info.status_val != 1: 
                print(sol.info.status)
                return None, None
            else:
                task_chain_objectivs.append(sol.info.obj_val)
                w.append(sol.x[self.N:])
                dq = sol.x[:self.N]

        
        return dq, task_chain_objectivs


    def _solve_part(self, tasks, wis, warmstart):
        qpD = QpDesc(self.N, self.r, self.lb, self.ub)
        qpD.addTask(tasks[-1])

        for i, t in enumerate(tasks[:-1]):
            qpD.addTask(t, wis[i])

        H = sparse.csc_matrix(qpD.H)
        g = qpD.q
        
        A = sparse.csc_matrix(qpD.A)
        ubA = qpD.ub
        lbA = qpD.lb


        m = osqp.OSQP()
        m.setup(P=H, q=g, A=A, l=lbA, u=ubA, verbose=False)

        if warmstart is not None:
            m.warm_start(x=np.r_[warmstart, np.zeros(g.size - warmstart.size)])

        sol = m.solve()
        return sol



if __name__ == "__main__":
    q = QpDesc(5, 3)
    print(q.A)
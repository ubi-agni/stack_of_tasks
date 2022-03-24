#!/usr/bin/env python

from sympy import E
from task import TaskDesc
import numpy as np
from scipy import sparse
import osqp

class QpDesc:
    def __init__(self, N, rho) -> None:
        self.N = N
        
        self._P = np.identity(N) * rho
        self._q = np.zeros(self.N)

        self._A = np.zeros((0,N))

        self._lbA = np.zeros(0)
        self._ubA = np.zeros(0)
        
        self._AStatic = np.zeros((0,N))
        self._lbStatic = np.zeros(0)
        self._ubStatic = np.zeros(0)
        
        self._slacks = np.zeros(0)

    @property
    def H(self):
        nSlack = self._A.shape[0] - self._slacks.size
        H = np.identity(self.N + nSlack)
        H[:self.N, :self.N] = self._P
        return H

    @property
    def q(self):
        nSlack = self._A.shape[0] - self._slacks.size
        return np.r_[self._q, np.zeros(nSlack)]

    @property
    def A(self):
        nSlack = self._A.shape[0] - self._slacks.size

        sMat = np.r_[np.zeros( (self._slacks.size + self._lbStatic.size , nSlack) ) , -np.identity(nSlack)]
        return np.c_[ np.r_[np.identity(self._lbStatic.size), self._A ], sMat] 

    @property
    def lb(self):
        nSlack = self._lbA.size - self._slacks.size
        return np.r_[self._lbStatic, self._lbA] + np.r_[np.zeros(self._lbStatic.size), self._slacks, np.zeros(nSlack) ]

    @property
    def ub(self):
        nSlack = self._ubA.size - self._slacks.size


        return np.r_[self._ubStatic, self._lbA] + np.r_[ np.zeros(self._ubStatic.size), self._slacks, np.zeros(nSlack) ]

    def addTask(self, desc: TaskDesc):
        At, ut, lt = desc.unpack()

        self._A = np.r_[self._A, At]  
        self._lbA = np.r_[self._lbA, lt]
        self._ubA = np.r_[self._ubA, ut]
    
    def add_slack(self, slack):
        self._slacks = np.r_[self._slacks, slack]

    def add_task_as_static(self, desc):
        self.add_static_bounds(desc.lower, desc.upper)

    def add_static_bounds(self, lb, ub):
        self._lbStatic = np.r_[self._lbStatic, lb]
        self._ubStatic = np.r_[self._ubStatic, ub]




class Solver:
    def __init__(self, N, lb, ub, rho) -> None:
        self.N = N

        self.lb = lb
        self.ub = ub
        self.r = rho

        self.qp_desc = None

    def solve_sot(self, task_descs, warmstart=None):
        assert len(task_descs)>0

        self.qp_desc = QpDesc(self.N, self.r)
        self.qp_desc.add_static_bounds(self.lb, self.ub)


        task_chain_objectivs = []
        dq = warmstart

        for t in task_descs:
            self.qp_desc.addTask(t)
            sol =  self._solve_qp(warmstart)

            if sol.info.status_val != 1: 
                print(sol.info.status)
                return None, None
            else:
                task_chain_objectivs.append(sol.info.obj_val)
                self.qp_desc.add_slack(sol.x[self.N:])
                dq = sol.x[:self.N]

        return dq, task_chain_objectivs


    def _solve_qp(self, warmstart=None):

        H = sparse.csc_matrix(self.qp_desc.H)
        g = self.qp_desc.q
        
        A = sparse.csc_matrix(self.qp_desc.A)
        ubA = self.qp_desc.ub
        lbA = self.qp_desc.lb


        m = osqp.OSQP()
        m.setup(P=H, q=g, A=A, l=lbA, u=ubA, verbose=False)

        if warmstart is not None:
            m.warm_start(x=np.r_[warmstart, np.zeros(g.size - warmstart.size)])

        sol = m.solve()
        return sol



if __name__ == "__main__":
    q = QpDesc(5, 3)
    print(q.A)
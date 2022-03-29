#!/usr/bin/env python

from sympy import E
from task import TaskDesc
import numpy as np
from scipy import sparse
import osqp

class QpDesc:
    def __init__(self, N, rho) -> None:
        self.N = N
        

        # initialized all needed matrices and vectors
        self._P = np.identity(N) * rho
        self._q = np.zeros(self.N)

        self._A_opt = np.zeros((0,N)) # A matrix aller bereits optimierten tasks
        self._lbA_opt = np.zeros(0)   # lower und upper    " 
        self._ubA_opt = np.zeros(0)

        self._A_nopt = np.zeros((0,N)) # A matrix aller tasks, die im nächsten schritt optimiert werden müssen
        self._lbA_nopt = np.zeros(0)
        self._ubA_nopt = np.zeros(0)

    @property
    def sN(self):
        """Number of slack variables"""
        return self._lbA_nopt.size

    @property
    def H(self):
        H = np.identity(self.N + self.sN)
        H[:self.N, :self.N] = self._P
        return H

    @property
    def q(self):
        return np.r_[self._q, np.zeros(self.sN)]

    @property
    def A(self):

        #  Matrix A aller Tasks
        A = np.r_[
            self._A_opt,    
            self._A_nopt,        
            -self._A_nopt,
            #np.zeros((self.sN, self.N))
            ]

        # bestimmt von welche Tasks die slack-variablen abgezogen werden
        sMat = np.r_[
            np.zeros((self._A_opt.shape[0], self.sN)),
            -np.identity(self.sN), 
            -np.identity(self.sN),
            #np.identity(self.sN)
            ]

        return np.c_[A,sMat]

    @property
    def lb(self):
        return np.r_[
            self._lbA_opt, 
            np.full(2*self.sN, np.NINF),  # lower bound der optimierung ist immer -INF
            #np.ones(self.sN) * 10e-5
            ]

    @property
    def ub(self):
        return np.r_[
            self._ubA_opt, 
            self._ubA_nopt, 
            -self._lbA_nopt, # da es nur upper bounds gibt, wird aus lower upper bound 
            #10e20 * np.ones(self.sN)
            ]

    def add_task(self, desc: TaskDesc):
        At, ut, lt = desc.unpack()

        self._A_nopt = np.r_[self._A_nopt, At]
        self._ubA_nopt = np.r_[self._ubA_nopt, ut]
        self._lbA_nopt = np.r_[self._lbA_nopt, lt]
    
    def add_slack(self, slack_vect):
        assert slack_vect.size == self._lbA_nopt.size == self._ubA_nopt.size

        self._A_opt = np.r_[
            self._A_opt, 
            self._A_nopt
            ]
        
        self._ubA_opt = np.r_[
            self._ubA_opt, 
            self._ubA_nopt+slack_vect
            ]
        
        self._lbA_opt = np.r_[
            self._lbA_opt,
            self._lbA_nopt-slack_vect
            ]

        # wird slack negativ, kann l > u werden -> bounds tauschen
        v = self._lbA_opt > self._ubA_opt 
        self._ubA_opt[v], self._lbA_opt[v] = self._ubA_opt[v], self._lbA_opt[v]

        self._A_nopt = np.zeros((0,self.N))
        self._ubA_nopt = np.zeros(0)
        self._lbA_nopt = np.zeros(0)

    def add_task_hard(self, desc):
        self._ubA_opt = np.r_[self._ubA_opt, desc.upper]
        self._lbA_opt = np.r_[self._lbA_opt, desc.lower]
        self._A_opt = np.r_[self._A_opt, desc.A]

    def add_static_bounds(self, lb, ub):
        assert lb.size ==  ub.size == self.N
        self._A_opt = np.r_[self._A_opt, np.identity(lb.size)]
        self._ubA_opt = np.r_[self._ubA_opt, ub]
        self._lbA_opt = np.r_[self._lbA_opt, lb]




class Solver:
    def __init__(self, N, lb, ub, rho) -> None:
        self.N = N

        self.qp_desc = QpDesc(self.N, rho)
        self.qp_desc.add_static_bounds(lb, ub)

    def solve_sot(self, task_descs, warmstart=None):
        assert len(task_descs)>0

        task_chain_objectivs = []
        dq = warmstart

        for t in task_descs:
            self.qp_desc.add_task(t)
            sol =  self._solve_qp(dq)

            if sol.info.status_val < 0 : 
                print(sol.info.status)
                return None, None
            else:
                task_chain_objectivs.append(sol.info.obj_val)
                self.qp_desc.add_slack(sol.x[self.N:])
                dq = sol.x[:self.N]

        return dq, task_chain_objectivs


    def _solve_qp(self, warmstart=None):

        H = self.qp_desc.H
        g = self.qp_desc.q
        A = self.qp_desc.A
        ub = self.qp_desc.ub
        lb = self.qp_desc.lb

        H = sparse.csc_matrix(H)
        A = sparse.csc_matrix(A)

        m = osqp.OSQP()
        m.setup(P=H, q=g, A=A, l=lb, u=ub, verbose=False)

        if warmstart is not None:
            # slackvariablen mit 0 warmstarten, etwas anderes sinnvoller?
            m.warm_start(x=np.r_[warmstart, np.full(g.size - warmstart.size, 0)])

        sol = m.solve()
        return sol



if __name__ == "__main__":
    q = QpDesc(5, 3)
    print(q.A)
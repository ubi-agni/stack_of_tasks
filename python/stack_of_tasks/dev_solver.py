#!/usr/bin/env python3
import numpy as np
from scipy import sparse

T = 3
xmin = np.ones(5) * 2
umin = np.ones(4)
box_lower = np.hstack([np.kron(np.ones(T), xmin.T), np.kron(np.ones(T), umin.T)])
Bd = np.random.randn(3, 5)
nx, nu = Bd.shape


Aineq = sparse.eye((T + 1) * nx + T * nu)
s = sparse.csc_matrix((1, (T + 1) * nx + T * nu))
a = np.zeros((T + 1) * nx + T * nu)
print(nx, (T + 1) * nx)

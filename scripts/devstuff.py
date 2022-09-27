#!/usr/bin/env python3
import cProfile

import numpy as np


def vs(mat):
    r = np.zeros((0, 20))
    for m in mat:
        r = np.vstack([r, m])
    return


def getMatrix(mat, indices):
    return mat[indices]


def setMatrix(mat, x):
    mat[x : x + 10, :] = x


def test1():
    matrices = []
    for x in range(0, 1000):
        matrices.append(np.ones((10, 20)) * x)

    def test():
        for _ in range(10000):
            vs(matrices)

    cProfile.runctx("test()", None, locals())


def test2():
    matrix = np.zeros((10000, 20))

    def test():
        for _ in range(10000):
            for x in range(1000):
                setMatrix(matrix, x)

    cProfile.runctx("test()", None, locals())


def test3():

    m = np.zeros((20000, 20))
    inds = np.r_[tuple([np.r_[x : x + 10] for x in range(1, 20000, 20)])]

    def test():
        for _ in range(10000):
            for x in range(1000):
                setMatrix(m, x * 2)
            getMatrix(m, inds)

    cProfile.runctx("test()", None, locals())


test1()
# test2()
# test3()

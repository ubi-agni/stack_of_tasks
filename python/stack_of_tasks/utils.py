from typing import Iterable

import numpy as np


class Callback(list):
    def __call__(self, *args) -> None:
        for listener in self:
            listener(*args)


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def prettyprintMatrix(matrix):

    str_matrix = []
    maxL = 0
    for row in matrix:
        col = []
        if isinstance(row, Iterable):
            for x in row:
                s = f"{x:.2e}"
                maxL = max(len(s), maxL)
                col.append(s)
        else:
            s = f"{row:.2e}"
            maxL = max(len(s), maxL)
            col.append(s)
        str_matrix.append(col)

    s = ""
    for r in str_matrix[:-1]:
        for x in r[:-1]:
            s += f"{x :^{maxL}}│"
        s += f"{r[-1] :^{maxL}}"
        s += "\n"
        s += ("─" * maxL + "┼") * (len(r) - 1) + "─" * maxL
        s += "\n"

    for r in str_matrix[-1][:-1]:
        s += f"{r :^{maxL}}│"
    s += f"{str_matrix[-1][-1] :^{maxL}}"

    return s

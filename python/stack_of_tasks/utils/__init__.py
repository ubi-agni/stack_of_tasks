from numpy.typing import NDArray


def prettyprintMatrix(matrix: NDArray, fmt=".2"):
    if len(matrix.shape) == 1:
        s = [f"{e:{fmt}}" for e in matrix]
        l = max(map(len, s))
        return "\n".join(f"{e:^{l}}" for e in s)
    elif len(matrix.shape) == 2:
        s = [[f"{e:{fmt}}" for e in row] for row in matrix]
        l = [max(map(len, col)) for col in zip(*s)]
        f = [[f"{e:^{f}}" for e in r] for f, r in zip(l, zip(*s))]
        return "\n".join("\t".join(col) for col in zip(*f))

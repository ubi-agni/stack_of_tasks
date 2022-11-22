import numpy as np


def skew(w):
    """
    Generates skew-symmetric matrix.
    Usefull for cross product as matrix operation.

    Args:
        w (NDArray): Left side vector of cross product

    Returns:
        NDArray: Matrix
    """
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def adjoint(T, inverse=False):
    """
    Generates the adjoint matrix for given Transform T.

    Args:
        T (NDArray): Transform matrix.
        inverse (bool, optional): Inverse adjoint. Defaults to False.

    Returns:
        NDArray: Adjoint matrix
    """
    if T.shape == (4, 4):
        R = T[0:3, 0:3]
        p = T[0:3, 3]
    elif T.shape == (3, 3):
        R = T
        p = np.zeros(3)
    else:
        R = np.identity(3)
        p = T
    r = np.zeros((6, 6))

    if inverse:
        r[:3, :3] = r[3:, 3:] = R.T
        r[:3, 3:] = R.T @ skew(-p)
    else:
        r[:3, :3] = r[3:, 3:] = R
        r[:3, 3:] = skew(p) @ R

    return r

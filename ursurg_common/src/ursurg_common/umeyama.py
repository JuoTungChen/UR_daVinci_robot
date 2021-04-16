import numpy as np


def umeyama(x, y, with_scaling=True):
    x = np.asarray(x)
    y = np.asarray(y)

    assert x.ndim == 2
    assert y.ndim == 2
    assert x.shape == y.shape

    n, m = x.shape

    ux = x.mean(axis=0)
    uy = y.mean(axis=0)
    dx = x - ux
    dy = y - uy

    sigma = dy.T @ dx / n
    U, d, V_t = np.linalg.svd(sigma)
    S = np.eye(m)

    if np.linalg.det(U) * np.linalg.det(V_t.T) < 0:
        S[m-1,m-1] = -1

    T = np.eye(m + 1)
    T[:m,:m] = U @ S @ V_t

    if with_scaling:
        c = (d * S.diagonal()).sum() / dx.var(axis=0).sum()
        T[:m,m] = uy - c * T[:m,:m] @ ux
        T[:m,:m] *= c
    else:
        T[:m,m] = uy - T[:m,:m] @ ux

    return T


if __name__ == '__main__':
    x = [(0, 0), (1, 0), (0, 2)]
    y = [(0, 0), (-1, 0), (0, 2)]

    T = umeyama(x, y)
    print(T)

    Tans = np.array([
        [0.832,  0.555, -0.8],
        [-0.555, 0.832, 0.4],
        [0, 0, 1],
    ])
    Tans[:2,:2] *= 0.721
    print(Tans)

    assert np.allclose(T, Tans, atol=1e-3)

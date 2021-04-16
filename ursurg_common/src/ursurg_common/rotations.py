import math
import numpy as np


def Rx(a):
    """Basic rotation about the x axis with angle a"""
    ca = math.cos(a)
    sa = math.sin(a)

    return np.array([
        [1,   0,   0  ],
        [0,   ca,  -sa],
        [0,   sa,  ca ],
    ])


def Ry(a):
    """Basic rotation about the y axis with angle a"""
    ca = math.cos(a)
    sa = math.sin(a)

    return np.array([
        [ca,  0,   sa ],
        [0,   1,   0  ],
        [-sa, 0,   ca ],
    ])


def Rz(a):
    """Basic rotation about the z axis with angle a"""
    ca = math.cos(a)
    sa = math.sin(a)

    return np.array([
        [ca,  -sa, 0  ],
        [sa,  ca,  0  ],
        [0,   0,   1  ],
    ])


def cross_matrix(v):
    """Return 3x3 skew-symmetric matrix from a 3-dimensional vector.

    Can be used to represent cross products as matrix multiplication, e.g.
    v x u = [v]_x u, where [v]_x is a skew-symmetric matrix.
    """
    return np.array([
        [0,     -v[2],  v[1]],
        [v[2],   0,    -v[0]],
        [-v[1],  v[0],  0],
    ])


def rodrigues(axis, angle):
    """Return the 3x3 rotation matrix through 'angle' about 'axis'"""
    K = cross_matrix(axis)
    return np.identity(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K**2


def quaternion_average(quaternions, weights=None):
    """Average a set of quaternions

    quaternions: Nx4 array of quaternions arranged as (w,x,y,z)
    weights: Array of N weights

    F. Landis Markley, Yang Cheng, John Lucas Crassidis and Yaakov Oshman.
    Journal of Guidance, Control, and Dynamics, Vol. 30, No. 4 (2007), pp. 1193-1197
    """
    weights = np.ones(len(quaternions)) if weights is None else weights
    return np.linalg.eigh(np.einsum('ij,ik,i->...jk', quaternions, quaternions, weights))[1][:,-1]

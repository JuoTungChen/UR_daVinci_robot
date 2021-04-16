import numpy as np


def fit_plane(points):
    """
    Fit orthogonal least squares plane to 3D points.

    The singular vector corresponding to the smallest eigenvalue is normal to
    the plane and the plane contains the centroid of the data.

    """

    # Centroid of the data
    C = np.mean(points, axis=0)

    # SVD of the data with the centroid subtracted
    U, s, Vh = np.linalg.svd(points - C)

    return Vh[2,:], C  # (a, b, c), (x0, y0, z0)

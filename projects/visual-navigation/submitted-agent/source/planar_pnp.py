"""Experimental upright-camera pose refinement from image correspondences.

Constrain motion to horizontal translation and yaw. The objective is image
reprojection error, so distant reconstructed walls do not dominate rigid fits.
"""
import cv2
import numpy as np
from scipy.optimize import least_squares
from visual_pose import wall_points


def project(points, pose, K):
    x, z, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    transformed = points@np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])+[x, 0, z]
    q = transformed@K.T
    uv = q[:, :2]/np.maximum(q[:, 2:3], .02)
    return uv, transformed[:, 2]


def fit(points, pixels, K, initial):
    points, pixels = np.asarray(points, float), np.asarray(pixels, float)
    if len(points) < 12 or not np.isfinite(points).all() or not np.isfinite(pixels).all():
        return None
    def residual(value, mask):
        projected, _ = project(points[mask], value, K)
        return (projected-pixels[mask]).ravel()
    mask = np.ones(len(points), bool)
    value = np.array(initial[:3], float)
    for _ in range(3):
        fitted = least_squares(residual, value, args=(mask,), loss='soft_l1',
                               f_scale=2., max_nfev=60)
        value = fitted.x
        projected, depth = project(points, value, K)
        errors = np.linalg.norm(projected-pixels, axis=1)
        new_mask = (errors <= 3.) & (depth > .1)
        if new_mask.sum() < 12:
            return None
        if np.array_equal(mask, new_mask):
            break
        mask = new_mask
    if np.linalg.matrix_rank(fitted.jac) < 3 or np.linalg.cond(fitted.jac) > 1e6:
        return None
    if np.median(errors[mask]) > 1.5:
        return None
    return (*map(float, value), int(mask.sum()))


def from_points(image, first, second, K, initial=None):
    if len(first) < 12:
        return None
    points, valid = wall_points(image, first, K)
    valid &= points[:, 2] < 15
    if valid.sum() < 12:
        return None
    points, target = points[valid], np.asarray(second)[valid]
    if initial is None:
        ok, rv, tv, inliers = cv2.solvePnPRansac(points, target, K, None,
            iterationsCount=300, reprojectionError=3, confidence=.995, flags=cv2.SOLVEPNP_EPNP)
        if not ok or inliers is None or len(inliers) < 12:
            return None
        R, _ = cv2.Rodrigues(rv)
        initial = (tv[0, 0], tv[2, 0], np.arctan2(R[0, 2], R[2, 2]))
    return fit(points, target, K, initial)

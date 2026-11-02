"""Robust planar registration of wall points reconstructed in both images."""

import cv2
import numpy as np
from visual_pose import wall_points


def estimate(a, b, image_a, image_b, camera, matcher):
    p, d = a
    q, e = b
    if d is None or e is None or len(e) < 2:
        return None
    matches = [
        m
        for pair in matcher.knnMatch(d, e, k=2)
        if len(pair) == 2
        for m, n in [pair]
        if m.distance < 0.7 * n.distance
    ]
    if len(matches) < 8:
        return None
    pa, va = wall_points(image_a, p[[m.queryIdx for m in matches]], camera)
    pb, vb = wall_points(image_b, q[[m.trainIdx for m in matches]], camera)
    keep = va & vb & (pa[:, 2] < 15) & (pb[:, 2] < 15)
    if keep.sum() < 8:
        return None
    source, target = pa[keep][:, (0, 2)], pb[keep][:, (0, 2)]
    affine, inliers = cv2.estimateAffinePartial2D(
        source,
        target,
        method=cv2.RANSAC,
        ransacReprojThreshold=0.15,
        maxIters=300,
        confidence=0.995,
    )
    if affine is None or inliers is None or inliers.sum() < 8:
        return None
    scale = np.hypot(affine[0, 0], affine[0, 1])
    if not 0.9 < scale < 1.1:
        return None
    keep = inliers.ravel() > 0
    first, second = source[keep], target[keep]
    u, _, vt = np.linalg.svd((first - first.mean(axis=0)).T @ (second - second.mean(axis=0)))
    row_rotation = u @ vt
    if np.linalg.det(row_rotation) < 0:
        return None
    translation = np.median(second - first @ row_rotation, axis=0)
    yaw = float(np.arctan2(row_rotation[1, 0], row_rotation[0, 0]))
    residual = np.linalg.norm(second - first @ row_rotation - translation, axis=1)
    if np.median(residual) > 0.12:
        return None
    return float(translation[0]), float(translation[1]), yaw, int(keep.sum())

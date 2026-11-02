"""Relative pose from wall features and the visible floor boundary.

All depths are in camera-height units: no map, simulator pose, or metric goal is used.
The scene assumption is a flat floor and vertical walls, as in exploration images.
"""

import cv2
import numpy as np


def wall_points(image, pixels, K):
    h, w = image.shape[:2]
    yy = int(K[1, 2]) + 3
    # The two flat floor colours (unlit and shadowed) in the supplied renderer.
    b, g, r = cv2.split(image.astype(np.float32))
    floor = ((abs(b - 232) < 12) & (abs(g - 232) < 12) & (abs(r - 232) < 12)) | (
        (abs(b - 217) < 14) & (abs(g - 185) < 14) & (abs(r - 163) < 14)
    )
    # Ignore isolated JPEG pixels while preserving wall edges.
    floor = cv2.morphologyEx(floor.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    boundary = np.full(w, np.nan)
    for x in range(w):
        if not floor[-2, x]:
            continue
        non = np.flatnonzero(~floor[yy : h - 2, x].astype(bool))
        if len(non):
            boundary[x] = yy + non[-1] + 1
    # Near walls can exit the bottom at either image borders or interior columns.
    # Extrapolate each straight ground-edge segment into its adjacent missing gap.
    visible = np.isfinite(boundary)
    segments = np.split(
        np.flatnonzero(visible), np.flatnonzero(np.diff(np.flatnonzero(visible)) > 1) + 1
    )
    for run in segments:
        if len(run) < 8:
            continue
        for side in ("left", "right"):
            edge = run[0] if side == "left" else run[-1]
            sample = run[:8] if side == "left" else run[-8:]
            if boundary[edge] < h - 12:
                continue
            slope, intercept = np.polyfit(sample, boundary[sample], 1)
            if np.median(abs(slope * sample + intercept - boundary[sample])) > 1.5:
                continue
            direction = -1 if side == "left" else 1
            x = edge + direction
            while 0 <= x < w and not visible[x]:
                value = slope * x + intercept
                if value < h - 12:
                    break
                boundary[x] = value
                x += direction
    u = np.clip(np.rint(pixels[:, 0]).astype(int), 0, w - 1)
    bottom = boundary[u]
    z = K[1, 1] / (bottom - K[1, 2])
    points = np.column_stack(
        ((pixels[:, 0] - K[0, 2]) * z / K[0, 0], (pixels[:, 1] - K[1, 2]) * z / K[1, 1], z)
    )
    valid = np.isfinite(points).all(axis=1) & (z > 0) & (z < 50) & (pixels[:, 1] < bottom - 3)
    return points.astype(np.float32), valid


def relative_pose(ref_features, live_features, ref_image, K, matcher, ratio=0.7):
    p, d = ref_features
    q, e = live_features
    if d is None or e is None or len(e) < 2 or len(p) < 6:
        return None
    good = [
        m
        for pair in matcher.knnMatch(d, e, k=2)
        if len(pair) == 2
        for m, n in [pair]
        if m.distance < ratio * n.distance
    ]
    if len(good) < 8:
        return None
    a = p[[m.queryIdx for m in good]]
    b = q[[m.trainIdx for m in good]]
    points, valid = wall_points(ref_image, a, K)
    if valid.sum() < 8:
        return None
    ok, rv, tv, inliers = cv2.solvePnPRansac(
        points[valid],
        b[valid],
        K,
        None,
        iterationsCount=150,
        reprojectionError=3,
        confidence=0.995,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok or inliers is None or len(inliers) < 8:
        return None
    indices = inliers.ravel()
    ok, rv, tv = cv2.solvePnP(
        points[valid][indices],
        b[valid][indices],
        K,
        None,
        rv,
        tv,
        True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    R, _ = cv2.Rodrigues(rv)
    angle = float(np.arctan2(R[0, 2], R[2, 2]))
    t = tv.ravel()
    if abs(t[1]) > 0.5 or abs(R[1, 0]) > 0.15 or abs(R[1, 2]) > 0.15:
        return None
    return float(t[0]), float(t[2]), angle, len(indices)


def image_pose_error(reference, live, pose, camera_matrix):
    """Check a feature pose against textured wall pixels across the whole view."""
    height, width = reference.shape[:2]
    yy, xx = np.mgrid[4:height:4, 4:width:4]
    pixels = np.column_stack((xx.ravel(), yy.ravel())).astype(np.float32)
    points, valid = wall_points(reference, pixels, camera_matrix)
    gray = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY).astype(np.float32)
    detail = abs(cv2.Laplacian(gray, cv2.CV_32F))
    valid &= detail[pixels[:, 1].astype(int), pixels[:, 0].astype(int)] > 8
    x, z, yaw, _ = pose
    rotation = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    transformed = points @ rotation.T + [x, 0, z]
    projection = transformed @ camera_matrix.T
    with np.errstate(invalid="ignore", divide="ignore"):
        uv = projection[:, :2] / projection[:, 2:3]
    inside = valid & (transformed[:, 2] > 0) & np.isfinite(uv).all(axis=1)
    inside &= (uv[:, 0] >= 2) & (uv[:, 0] < width - 2) & (uv[:, 1] >= 2) & (uv[:, 1] < height - 2)
    if inside.sum() < 80:
        return None
    a = cv2.GaussianBlur(reference, (5, 5), 0).astype(np.float32)
    b = cv2.GaussianBlur(live, (5, 5), 0).astype(np.float32)
    selected = pixels[inside].astype(int)
    projected = uv[inside].astype(np.float32)
    colors = cv2.remap(b, projected[:, 0][None], projected[:, 1][None], cv2.INTER_LINEAR)[0]
    errors = np.mean(abs(a[selected[:, 1], selected[:, 0]] - colors), axis=1) / 255
    bands = []
    for band in range(4):
        mask = (selected[:, 0] * 4 // width) == band
        if mask.sum() >= 20:
            bands.append(float(np.median(errors[mask])))
    return (
        float(np.mean(errors)),
        max(bands, default=1.0),
        len(bands),
        float(inside.sum() / max(1, valid.sum())),
    )


def image_pose_consistent(reference, live, pose, camera_matrix):
    """Require agreement beyond the single textured patch that fitted PnP."""
    x, z, yaw, count = pose
    inverse = (
        -np.cos(yaw) * x + np.sin(yaw) * z,
        -np.sin(yaw) * x - np.cos(yaw) * z,
        -yaw,
        count,
    )
    for a, b, estimate in ((reference, live, pose), (live, reference, inverse)):
        error = image_pose_error(a, b, estimate, camera_matrix)
        if error is None:
            return False
        mean_error, worst_band, bands, coverage = error
        if mean_error > 0.05 or worst_band > 0.15 or bands < 3 or coverage < 0.65:
            return False
    return True

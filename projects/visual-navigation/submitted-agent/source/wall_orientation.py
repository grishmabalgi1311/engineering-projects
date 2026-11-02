"""Camera orientation modulo 90 degrees, inferred from visible wall/floor edges."""

import numpy as np
from visual_pose import wall_points


def wall_axis(image, camera):
    columns = np.linspace(4, image.shape[1] - 5, 80)
    pixels = np.column_stack((columns, np.full(len(columns), camera[1, 2])))
    points, valid = wall_points(image, pixels, camera)
    xz = points[:, (0, 2)]
    angles, weights = [], []
    for i in range(len(columns) - 7):
        if not valid[i : i + 8].all():
            continue
        selected = xz[i : i + 8]
        if np.max(selected[:, 1]) > 25:
            continue
        centered = selected - selected.mean(axis=0)
        _, singular, vectors = np.linalg.svd(centered, full_matrices=False)
        if singular[0] < 0.15 or singular[1] > 0.025 or singular[1] > singular[0] * 0.04:
            continue
        direction = vectors[0]
        angles.append(np.arctan2(direction[0], direction[1]))
        weights.append(min(float(singular[0]), 2.0))
    if len(angles) < 4:
        return None
    angles, weights = np.array(angles), np.array(weights)
    mean = np.sum(weights * np.exp(4j * angles)) / np.sum(weights)
    if abs(mean) < 0.85:
        return None
    errors = abs(np.angle(np.exp(4j * angles) / mean)) / 4
    keep = errors < 0.04
    if keep.sum() < 4:
        return None
    mean = np.sum(weights[keep] * np.exp(4j * angles[keep])) / np.sum(weights[keep])
    return float(np.angle(mean) / 4), float(abs(mean))

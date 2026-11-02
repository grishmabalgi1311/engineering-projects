"""Image-derived planar pose graph. No simulator state or metric map is read."""

import hashlib
import pickle
from pathlib import Path

import cv2
import numpy as np
from planar_odometry import estimate
from scipy.sparse import coo_matrix, vstack
from scipy.sparse.linalg import lsqr
from visual_pose import relative_pose
from wall_orientation import wall_axis


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, s], [-s, c]])


def build_map(agent, camera):
    """Fit image odometry to independent, image-verified loop constraints."""
    signature = hashlib.sha256(
        repr((agent.target_cache_signature, camera.tolist())).encode()
        + Path(__file__).read_bytes()
        + Path(__file__).with_name("visual_pose.py").read_bytes()
        + Path(__file__).with_name("planar_odometry.py").read_bytes()
        + Path(__file__).with_name("wall_orientation.py").read_bytes()
    ).hexdigest()
    path = agent.target_cache_path.parent / f"spatial_map_{signature[:16]}.pkl"
    if path.exists():
        saved = pickle.loads(path.read_bytes())
        if saved["signature"] == signature:
            print("spatial map cache hit", flush=True)
            return saved
    size = len(agent.frames)
    edges = []
    odometry = np.zeros((size, 3))
    counts = {"visual": 0, "fallback": 0, "loops": 0}
    detector = cv2.SIFT_create(nfeatures=1200)
    axes = []
    for image_path, _ in agent.frames:
        value = wall_axis(cv2.imread(str(image_path)), camera)
        axes.append(value[0] if value is not None else np.nan)
    axes = np.array(axes)
    initial_axis = axes[agent.routes[0][0]]
    if not np.isfinite(initial_axis):
        initial_axis = 0.0

    def features(image):
        keypoints, descriptors = detector.detectAndCompute(image, None)
        return np.float32([k.pt for k in keypoints]), descriptors

    def measure(a, b):
        image = cv2.imread(str(agent.frames[a][0]))
        live = cv2.imread(str(agent.frames[b][0]))
        pose = estimate(agent.features[a], agent.features[b], image, live, camera, agent.bf)
        if (
            pose is not None
            and pose[3] >= 12
            and np.hypot(pose[0], pose[1]) < 2.5
            and abs(pose[2]) < 1.3
        ):
            return pose
        live_features = features(live)
        best = pose
        for scale in (1.0, 1.5, 2.0, 0.75, 0.5):
            warped = cv2.resize(image, None, fx=scale, fy=1.0)
            points, descriptors = features(warped)
            if len(points):
                points[:, 0] /= scale
            candidate = estimate(
                (points, descriptors), live_features, image, live, camera, agent.bf
            )
            if candidate is None:
                candidate = relative_pose(
                    (points, descriptors), live_features, image, camera, agent.bf
                )
            if candidate is not None and (
                np.hypot(candidate[0], candidate[1]) > 2.5 or abs(candidate[2]) > 1.3
            ):
                candidate = None
            if candidate is not None and (best is None or candidate[3] > best[3]):
                best = candidate
            if best is not None and best[3] >= 20:
                break
        return best

    def bearing_rotation(a, b):
        first = cv2.imread(str(agent.frames[a][0]))
        second = cv2.imread(str(agent.frames[b][0]))
        p, d = features(first)
        q, e = features(second)
        if d is None or e is None or len(e) < 2:
            return None
        matches = [
            m
            for pair in agent.bf.knnMatch(d, e, k=2)
            if len(pair) == 2
            for m, n in [pair]
            if m.distance < 0.7 * n.distance
        ]
        if len(matches) < 4:
            return None
        x = p[[m.queryIdx for m in matches], 0]
        y = q[[m.trainIdx for m in matches], 0]
        angles = np.arctan((y - camera[0, 2]) / camera[0, 0]) - np.arctan(
            (x - camera[0, 2]) / camera[0, 0]
        )
        counts = (abs(angles[:, None] - angles[None, :]) < 0.08).sum(axis=1)
        i = int(np.argmax(counts))
        keep = abs(angles - angles[i]) < 0.08
        return float(np.median(angles[keep])) if keep.sum() >= 4 else None

    for route in agent.routes:
        for a, b in zip(route, route[1:], strict=False):
            pose = measure(a, b)
            names = agent.frames[a][1]
            linear = ("FORWARD" in names) != ("BACKWARD" in names)
            turning = ("LEFT" in names) != ("RIGHT" in names)
            valid = (
                pose is not None
                and pose[3] >= 10
                and np.hypot(pose[0], pose[1]) < 2.5
                and abs(pose[2]) < 1.3
            )
            if valid:
                x, z, yaw, _ = pose
                # A capture interval can straddle two commands, so its action
                # label must not override measured translation or rotation.
                weight = 1.0
                counts["visual"] += 1
            else:
                yaw = (0.3 if "LEFT" in names else -0.3) if turning else 0.0
                z = (-1.0 if "FORWARD" in names else 1.0) if linear else 0.0
                x = 0.0
                if linear and "BACKWARD" in names:
                    yaw = -yaw
                visual_rotation = bearing_rotation(a, b)
                if visual_rotation is not None:
                    yaw = visual_rotation
                    if abs(yaw) > 0.35:
                        z = 0.0
                weight = 0.15
                counts["fallback"] += 1
            # Camera-reference to camera-live transform -> world displacement
            # expressed in the reference frame.
            delta_angle = -yaw
            delta = -rotation(delta_angle) @ [x, z]
            edges.append((a, b, float(delta[0]), float(delta[1]), delta_angle, weight))
            odometry[b, 2] = odometry[a, 2] + delta_angle
            if np.isfinite(axes[b]):
                error = (initial_axis - axes[b] - odometry[b, 2] + np.pi / 4) % (
                    np.pi / 2
                ) - np.pi / 4
                odometry[b, 2] += error
            odometry[b, :2] = odometry[a, :2] + rotation(odometry[a, 2]) @ delta
        print("mapped trajectory", route[0], len(route), counts, flush=True)

    for a, b, _, _ in agent.route_connections:
        pose = measure(a, b)
        if pose is None:
            continue
        x, z, yaw, _ = pose
        delta = -rotation(-yaw) @ [x, z]
        edges.append((a, b, float(delta[0]), float(delta[1]), -yaw, 8.0))
        counts["loops"] += 1

    # Every public drive begins at the common spawn, but its first captured
    # frame may already have rotated. Measure that orientation from images.
    first = agent.routes[0][0]
    for route in agent.routes[1:]:
        b = route[0]
        pose = measure(first, b)
        if pose is not None and pose[3] >= 12 and np.hypot(pose[0], pose[1]) < 0.5:
            angle = -pose[2]
            edges.append((first, b, 0.0, 0.0, angle, 10.0))
        else:

            def initial_turn(node):
                names = agent.frames[node][1]
                return 0.3 if "RIGHT" in names else -0.3 if "LEFT" in names else 0.0

            angle = initial_turn(b) - initial_turn(first)
            edges.append((first, b, 0.0, 0.0, angle, 10.0))

    a = np.array([e[0] for e in edges], dtype=int)
    b = np.array([e[1] for e in edges], dtype=int)
    delta = np.array([e[2:5] for e in edges])
    weights = np.sqrt([e[5] for e in edges])
    rows = np.arange(len(edges))
    matrix = coo_matrix(
        (np.r_[-weights, weights, 100.0], (np.r_[rows, rows, len(edges)], np.r_[a, b, first])),
        shape=(len(edges) + 1, size),
    ).tocsr()
    angles = odometry[:, 2].copy()
    nodes = np.flatnonzero(np.isfinite(axes))
    targets = (
        angles[nodes]
        + (initial_axis - axes[nodes] - angles[nodes] + np.pi / 4) % (np.pi / 2)
        - np.pi / 4
    )
    heading_prior = coo_matrix(
        (np.full(len(nodes), 2.0), (np.arange(len(nodes)), nodes)), shape=(len(nodes), size)
    ).tocsr()
    angle_matrix = vstack((matrix, heading_prior)).tocsr()
    for _ in range(5):
        residual = wrap(angles[b] - angles[a] - delta[:, 2])
        correction = lsqr(
            angle_matrix,
            np.r_[-residual * weights, -100 * angles[first], 2 * wrap(targets - angles[nodes])],
            atol=1e-8,
            btol=1e-8,
            iter_lim=10000,
        )[0]
        angles += correction
        if np.max(abs(correction)) < 1e-5:
            break
    c, s = np.cos(angles[a]), np.sin(angles[a])
    dx = c * delta[:, 0] + s * delta[:, 1]
    dz = -s * delta[:, 0] + c * delta[:, 1]
    x = lsqr(matrix, np.r_[dx * weights, 0], atol=1e-8, btol=1e-8, iter_lim=10000)[0]
    z = lsqr(matrix, np.r_[dz * weights, 0], atol=1e-8, btol=1e-8, iter_lim=10000)[0]
    poses = np.column_stack((x, z, angles))
    residual = poses[b, :2] - poses[a, :2] - np.column_stack((dx, dz))
    report = {
        "signature": signature,
        "poses": poses,
        "odometry": odometry,
        "edges": edges,
        "counts": counts,
        "edge_position_error": np.linalg.norm(residual, axis=1),
        "edge_heading_error": wrap(angles[b] - angles[a] - delta[:, 2]),
        "wall_axes": axes,
    }
    path.write_bytes(pickle.dumps(report))
    print(
        "spatial map built",
        counts,
        "edge residual quantiles",
        np.quantile(report["edge_position_error"], [0.5, 0.9, 0.99]),
        flush=True,
    )
    return report

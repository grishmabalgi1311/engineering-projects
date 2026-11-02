"""Image-verified route connections across all public exploration trajectories."""

import hashlib
import heapq
import pickle
from pathlib import Path

import cv2
import numpy as np
import visual_pose
from visual_word_candidates import candidates
from sklearn.neighbors import NearestNeighbors
from threadpoolctl import threadpool_limits


def shorten_route(agent, camera_matrix):
    """Find the shortest recorded route through verified same-place links."""
    key = tuple(camera_matrix.ravel())
    cached = getattr(agent, "_route_graph", None)
    digest = hashlib.sha256(
        repr((agent.target_cache_signature, key)).encode()
        + Path(__file__).read_bytes()
        + Path(visual_pose.__file__).read_bytes()
    ).hexdigest()
    cache_path = agent.target_cache_path.parent / f"route_graph_{digest[:16]}.pkl"
    agent.route_graph_cache_hit = "memory" if cached is not None and cached[0] == key else "miss"
    if agent.route_graph_cache_hit == "miss" and cache_path.exists():
        try:
            signature, graph, connections = pickle.loads(cache_path.read_bytes())
            if signature == digest and len(graph) == len(agent.frames):
                cached = (key, graph)
                agent._route_graph = cached
                agent.route_connections = connections
                agent.route_graph_cache_hit = "disk"
        except (OSError, ValueError, TypeError, EOFError, pickle.UnpicklingError):
            pass
    if cached is None or cached[0] != key:
        count = len(agent.frames)
        graph = [[] for _ in range(count)]
        positions = {}
        for trajectory, route in enumerate(agent.routes):
            for offset, node in enumerate(route):
                positions[node] = (trajectory, offset)
            for a, b in zip(route, route[1:], strict=False):
                graph[a].append((b, 1.0))
                graph[b].append((a, 1.0))
        small = np.array([cv2.resize(t, (10, 8)).flatten() for t in agent.thumbs], dtype=np.float32)
        with threadpool_limits(limits=1):
            neighbors = NearestNeighbors(n_neighbors=min(256, count), algorithm="brute").fit(small)
            neighbors = neighbors.kneighbors(small, return_distance=False)
        word_neighbors = candidates(agent)
        neighbors = [np.unique(np.concatenate((nearby, word_neighbors[i]))) for i, nearby in enumerate(neighbors)]
        images = {}
        poses = {}

        def geometry(a, b, minimum, distance, angle):
            if a not in images:
                images[a] = cv2.imread(str(agent.frames[a][0]))
            pose = visual_pose.relative_pose(
                agent.features[a], agent.features[b], images[a], camera_matrix, agent.bf
            )
            poses[(a, b)] = pose
            return (
                pose is not None
                and pose[3] >= minimum
                and np.hypot(pose[0], pose[1]) <= distance
                and abs(pose[2]) <= angle
            )

        links = 0
        agent.route_connections = []
        for a, nearby in enumerate(neighbors):
            if a % 250 == 0: print("expanded graph", a, "links", links, flush=True)
            ta, ia = positions[a]
            for b in nearby:
                b = int(b)
                tb, ib = positions[b]
                if b <= a or (ta == tb and abs(ia - ib) <= 5):
                    continue
                if b not in word_neighbors[a] and np.mean(abs(agent.thumbs[a] - agent.thumbs[b])) > 0.10:
                    continue
                n, residual, _ = agent.match(agent.features[a], agent.features[b])
                if n < 40 or residual > 20 or not geometry(a, b, 40, 0.25, 0.35):
                    continue
                supported = any(
                    0 <= ia + delta < len(agent.routes[ta])
                    and 0 <= ib + delta < len(agent.routes[tb])
                    and geometry(
                        agent.routes[ta][ia + delta],
                        agent.routes[tb][ib + delta],
                        25,
                        0.4,
                        0.4,
                    )
                    for delta in (-1, 1)
                )
                if not supported:
                    continue
                if b not in images:
                    images[b] = cv2.imread(str(agent.frames[b][0]))
                if not visual_pose.image_pose_consistent(
                    images[a], images[b], poses[(a, b)], camera_matrix
                ):
                    continue
                graph[a].append((b, 0.2))
                graph[b].append((a, 0.2))
                links += 1
                agent.route_connections.append((a, b, n, residual))
        agent._route_graph = (key, graph)
        try:
            temporary = cache_path.with_suffix(".tmp")
            temporary.write_bytes(pickle.dumps((digest, graph, agent.route_connections)))
            temporary.replace(cache_path)
        except OSError:
            pass  # An optional cache must not stop navigation.
        print("verified route connections", links, flush=True)
    else:
        graph = cached[1]
    print("route graph cache", agent.route_graph_cache_hit, flush=True)
    # Exploration trajectories begin at the SDK's common spawn. The controller
    # still measures the chosen first reference before advancing from it.
    queue = []
    distance = {}
    previous = {}
    for route in agent.routes:
        if route:
            distance[route[0]] = 0.0
            previous[route[0]] = None
            heapq.heappush(queue, (0.0, route[0]))
    while queue:
        cost, node = heapq.heappop(queue)
        if cost != distance[node]:
            continue
        if node == agent.goal:
            break
        for next_node, weight in graph[node]:
            value = cost + weight
            if value < distance.get(next_node, float("inf")):
                distance[next_node] = value
                previous[next_node] = node
                heapq.heappush(queue, (value, next_node))
    result = []
    node = agent.goal
    while node is not None:
        result.append(node)
        node = previous[node]
    result.reverse()
    print("visual route length", agent.end + 1, "->", len(result), flush=True)
    return result

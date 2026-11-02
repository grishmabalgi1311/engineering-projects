"""Experimental return connections verified with visible-overlap image checks.

Uses public recorded images only. Every sequential transform is measured in both
directions. Endpoints must close in both integrations before bypassing a detour.
"""
import hashlib
import pickle
from pathlib import Path
import cv2
import numpy as np
from planar_odometry import estimate
from visual_pose import relative_pose, image_pose_consistent
from visible_pose_check import supported
from spatial_map import rotation, wrap
from visual_return_route import proposals, search


class Verifier:
    def __init__(self, agent):
        self.agent = agent
        self.images = {}
        self.features = {}
        self.detector = cv2.SIFT_create(nfeatures=2000, contrastThreshold=.015)
        self.learned = None
        dependencies = ('visible_return_route.py', 'visible_pose_check.py',
                        'visual_pose.py', 'planar_odometry.py', 'learned_matching.py')
        self.signature = hashlib.sha256(
            repr((agent.target_cache_signature, agent.K.tolist())).encode()
            + b''.join(Path(__file__).with_name(name).read_bytes() for name in dependencies)
        ).hexdigest()
        self.cache = agent.target_cache_path.parent/f'return_motion_{self.signature[:16]}.pkl'
        self.measured = pickle.loads(self.cache.read_bytes()) if self.cache.exists() else {}

    def feature(self, node):
        if node not in self.images:
            image = cv2.imread(str(self.agent.frames[node][0]))
            kp, des = self.detector.detectAndCompute(image, None)
            self.images[node] = image
            self.features[node] = (np.float32([p.pt for p in kp]), des)
        return self.features[node]

    @staticmethod
    def consistent(forward, reverse):
        if forward is None or reverse is None or min(forward[3], reverse[3]) < 25:
            return False
        if max(np.linalg.norm(forward[:2]), np.linalg.norm(reverse[:2])) > 2.5:
            return False
        if max(abs(forward[2]), abs(reverse[2])) > 1.3:
            return False
        cycle = rotation(reverse[2])@np.array(forward[:2])+reverse[:2]
        return np.linalg.norm(cycle) <= .06 and abs(wrap(forward[2]+reverse[2])) <= .03

    def measure(self, first, last):
        key = (first, last)
        if key in self.measured:
            return self.measured[key]
        a, b = self.feature(first), self.feature(last)
        image, other = self.images[first], self.images[last]
        K, matcher = self.agent.K, self.agent.bf
        forward = estimate(a, b, image, other, K, matcher)
        reverse = estimate(b, a, other, image, K, matcher)
        method = 'dense_planar'
        if not self.consistent(forward, reverse):
            forward = relative_pose(a, b, image, K, matcher)
            reverse = relative_pose(b, a, other, K, matcher)
            method = 'dense_pnp'
            if not (self.consistent(forward, reverse)
                    and image_pose_consistent(image, other, forward, K)):
                if self.learned is None:
                    from learned_matching import Matcher
                    self.learned = Matcher(Path(__file__).resolve().parents[1]/'cache/learned_matcher', upscale=True)
                forward = self.learned.pose(image, other, K)
                reverse = self.learned.pose(other, image, K)
                method = 'learned_visible'
                if not (self.consistent(forward, reverse) and supported(image, other, forward, K)):
                    method = None
        result = None if method is None else (forward, reverse, method)
        self.measured[key] = result
        self.cache.write_bytes(pickle.dumps(self.measured))
        return result

    def verify(self, proposal):
        chain, _, _, travel = proposal
        measured = []
        for a, b in zip(chain, chain[1:]):
            edge = self.measure(a, b)
            if edge is None:
                return None
            measured.append(edge)
        position, backward = np.zeros(2), np.zeros(2)
        angle = back_angle = 0.
        for forward, _, _ in measured:
            position += rotation(angle)@(-rotation(-forward[2])@np.array(forward[:2]))
            angle = wrap(angle-forward[2])
        for _, reverse, _ in reversed(measured):
            backward += rotation(back_angle)@(-rotation(-reverse[2])@np.array(reverse[:2]))
            back_angle = wrap(back_angle-reverse[2])
        closure = float(np.linalg.norm(position+rotation(angle)@backward))
        if (max(np.linalg.norm(position), np.linalg.norm(backward)) > .2
                or closure > .12 or abs(wrap(angle+back_angle)) > .05):
            return None
        return dict(position=position, angle=angle, travel=travel, closure=closure,
                    reverse_position=backward, captures=len(chain)-1,
                    methods={name:sum(row[2] == name for row in measured)
                             for name in ('dense_planar', 'dense_pnp', 'learned_visible')})


def shorten(agent):
    verifier = Verifier(agent)
    graph = [list(edges) for edges in agent._route_graph[1]]
    digest = hashlib.sha256(repr((verifier.signature, agent.goal, graph,
                                agent.spatial_map['signature'])).encode()).hexdigest()
    cache = agent.target_cache_path.parent/f'visible_return_route_{digest[:16]}.pkl'
    if cache.exists():
        route, accepted, rejected = pickle.loads(cache.read_bytes())
    else:
        proposed, accepted, rejected = proposals(agent), {}, set()
        for _ in range(len(proposed)+1):
            trial = [list(edges) for edges in graph]
            for (a, b), (_, point, angle, _) in proposed.items():
                if (a, b) in rejected:
                    continue
                cost = max(.2, (np.linalg.norm(point)/.15+abs(angle)/.06)/5)
                trial[a].append((b, cost))
                trial[b].append((a, cost))
            route, _ = search(trial, [r[0] for r in agent.routes if r], agent.goal)
            pending = []
            for a, b in zip(route, route[1:]):
                key = (min(a, b), max(a, b))
                if key in proposed and key not in accepted and key not in rejected:
                    pending.append(key)
            if not pending:
                break
            for pair in pending:
                evidence = verifier.verify(proposed[pair])
                if evidence is None:
                    rejected.add(pair)
                else:
                    accepted[pair] = evidence
                print('VISIBLE RETURN candidate', pair, 'accepted', evidence is not None, flush=True)
        cache.write_bytes(pickle.dumps((route, accepted, rejected)))
    print('VISIBLE RETURN route', len(agent.route), '->', len(route),
          'accepted', len(accepted), 'rejected', len(rejected), flush=True)
    return route, accepted

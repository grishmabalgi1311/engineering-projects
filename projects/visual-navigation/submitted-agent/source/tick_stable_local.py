"""Follow local image-derived path geometry without a globally rigid map."""

import cv2
import numpy as np
from planar_odometry import estimate
from pose_agent import MyAgent as WorkingAgent
from spatial_map import build_map, rotation, wrap
from visual_pose import relative_pose
from vis_nav_sdk import Action


class MyAgent(WorkingAgent):
    def setup(self, info):
        super().setup(info)
        self.spatial_map = build_map(self, self.K)
        self.spatial_connections = []
        self.local_motion = {
            (a, b): (np.array([x, z]), angle, weight)
            for a, b, x, z, angle, weight in self.spatial_map["edges"]
        }
        for (a, b), (p, angle, weight) in list(self.local_motion.items()):
            self.local_motion.setdefault((b, a), (-rotation(-angle) @ p, -angle, weight))
        self.local_detector = cv2.SIFT_create(nfeatures=800)
        self.local_anchor = None
        self.local_pose = None
        self.local_previous = None
        self.local_command = None
        self.local_lost = 0
        self.local_parent_until = None
        self.local_final = False
        self.local_last_index = 0
        self.local_stalled = 0
        self.local_progress_step = None

    def parent_mode(self, obs, index, reason, until=None):
        print("local controller handoff", obs.step, index, reason, flush=True)
        self.index = min(index, self.end)
        self.local_parent_until = self.index if until is None else min(until, self.end)
        self.last_ref = None
        self.predicted = None
        self.previous_command = None
        self.local_anchor = None
        self.local_pose = None
        self.local_previous = None
        return super().act(obs)

    def localize(self, image, features):
        indices = list(range(max(0, self.index - 3), min(self.end + 1, self.index + 21)))
        nodes = [self.route[i] for i in indices]
        loss = np.mean(abs(self.thumbs[nodes] - self.thumb(image)), axis=(1, 2, 3))
        candidates = [indices[i] for i in np.argsort(loss)[:6]]
        if self.local_anchor is not None and self.local_anchor not in candidates:
            candidates.append(self.local_anchor)
        best = None
        for index in candidates:
            node = self.route[index]
            ref = cv2.imread(str(self.frames[node][0]))
            pose = relative_pose(self.features[node], features, ref, self.K, self.bf)
            if (
                pose is None
                or pose[3] < 20
                or np.hypot(pose[0], pose[1]) > 1.2
                or abs(pose[2]) > 0.8
            ):
                continue
            score = pose[3] / (1 + 8 * np.hypot(pose[0], pose[1]) + 2 * abs(pose[2]))
            if best is None or score > best[0]:
                best = (score, index, pose)
        return None if best is None else best[1:]

    def act(self, obs):
        if self.local_final:
            return super().act(obs)
        if self.local_parent_until is not None:
            if self.phase != "route":
                self.local_final = True
                return super().act(obs)
            if self.index <= self.local_parent_until:
                return super().act(obs)
            self.local_parent_until = None
            self.previous_command = None
            self.predicted = None
            self.local_command = None
        kp, des = self.local_detector.detectAndCompute(obs.image, None)
        features = (np.float32([p.pt for p in kp]), des)
        if self.local_previous is not None and self.local_pose is not None:
            image, previous = self.local_previous
            movement = estimate(previous, features, image, obs.image, self.K, self.bf)
            if (
                movement is not None
                and movement[3] >= 12
                and np.hypot(movement[0], movement[1]) < 1.2
                and abs(movement[2]) < 0.5
            ):
                translation = rotation(movement[2]) @ self.local_pose[:2] + movement[:2]
                self.local_pose = np.r_[translation, wrap(self.local_pose[2] + movement[2])]
            elif self.local_command is not None:
                action, ticks = self.local_command
                turning = int(bool(action & Action.RIGHT)) - int(bool(action & Action.LEFT))
                moving = int(bool(action & Action.FORWARD)) - int(bool(action & Action.BACKWARD))
                angle = -0.06 * turning * ticks * (-1 if moving < 0 else 1)
                self.local_pose[:2] = rotation(angle) @ self.local_pose[:2]
                self.local_pose[2] = wrap(self.local_pose[2] + angle)
                self.local_pose[1] -= 0.15 * ticks * moving
        correction = self.localize(obs.image, features)
        if correction is not None:
            self.local_anchor, pose = correction
            self.local_pose = np.array(pose[:3])
            self.index = max(self.index, self.local_anchor)
            self.local_lost = 0
        else:
            self.local_lost += 1
        self.local_previous = (obs.image.copy(), features)
        if self.index == self.local_last_index:
            self.local_stalled += 1
        else:
            self.local_stalled = 0
            self.local_last_index = self.index
            self.local_progress_step = obs.step
        if self.local_progress_step is None:
            self.local_progress_step = obs.step
        if obs.step-self.local_progress_step > 60:
            self.local_stalled = 0
            self.local_progress_step = obs.step
            return self.parent_mode(obs, self.index + 1, "local progress stalled")
        if self.local_pose is None or self.local_lost > 24:
            return self.parent_mode(obs, self.index, "visual reference recovery")
        if self.local_anchor >= self.end - 2 and np.linalg.norm(self.local_pose[:2]) < 0.8:
            self.local_final = True
            return self.parent_mode(obs, self.end, "precise goal alignment")
        position = np.zeros(2)
        angle = 0.0
        target = self.local_anchor
        blocked = False
        path = [position.copy()]
        for end in range(self.local_anchor + 1, min(self.end + 1, self.local_anchor + 81)):
            a, b = self.route[end - 1 : end + 1]
            edge = self.local_motion.get((a, b))
            if edge is None or edge[2] < 0.9:
                blocked = True
                break
            position += rotation(angle) @ edge[0]
            angle += edge[1]
            target = end
            path.append(position.copy())
        robot = -rotation(-self.local_pose[2]) @ self.local_pose[:2]
        path = np.asarray(path)
        lo = max(0, self.index - self.local_anchor - 2)
        hi = min(len(path), max(lo + 1, 21))
        closest = lo + int(np.argmin(np.linalg.norm(path[lo:hi] - robot, axis=1)))
        self.index = max(self.index, self.local_anchor + closest)
        endpoint = rotation(self.local_pose[2]) @ path[-1] + self.local_pose[:2]
        if (self.local_anchor+len(path)-1 == self.end
                and self.index >= self.end-2 and self.local_lost == 0
                and np.linalg.norm(endpoint) < .8):
            self.local_final = True
            return self.parent_mode(obs, self.end, "measured path reaches goal alignment")
        chosen = len(path) - 1
        for candidate in range(closest + 1, len(path)):
            if (
                np.linalg.norm(path[candidate] - robot) >= 0.8
                and self.local_anchor + candidate > self.index
            ):
                chosen = candidate
                break
        target = self.local_anchor + chosen
        position = path[chosen]
        vector = rotation(self.local_pose[2]) @ position + self.local_pose[:2]
        if blocked and np.linalg.norm(vector) < 0.4:
            return self.parent_mode(
                obs, target, "align before uncertain local motion edge", until=target + 1
            )
        if target <= self.local_anchor:
            return self.parent_mode(obs, self.index + 1, "no measured local path")
        forward = vector[1] >= 0
        bearing = (
            np.arctan2(vector[0], vector[1]) if forward else np.arctan2(-vector[0], -vector[1])
        )
        if abs(bearing) > 0.12:
            action = Action.RIGHT if bearing > 0 else Action.LEFT
            ticks = max(1, min(4, round(abs(bearing) / 0.06)))
        else:
            action = Action.FORWARD if forward else Action.BACKWARD
            ticks = 1
        # The public SDK supports simultaneous translation and steering.
        # Use one tick only on a shallow, well-localized correction.
        if (
            0.12 < abs(bearing) < 0.30
            and np.linalg.norm(vector) > 0.6
            and correction is not None
            and self.local_lost == 0
            and (not blocked or chosen < len(path) - 2)
            and abs(self.local_pose[2]) < 0.45
        ):
            steering = Action.RIGHT if bearing > 0 else Action.LEFT
            if not forward:
                steering = Action.LEFT if bearing > 0 else Action.RIGHT
            action = (Action.FORWARD if forward else Action.BACKWARD) | steering
            ticks = 1
        self.local_command = (action, ticks)
        if obs.step % 25 == 0:
            print(
                "local metric path",
                obs.step,
                self.index,
                "anchor",
                self.local_anchor,
                "target",
                target,
                flush=True,
            )
        return action, ticks

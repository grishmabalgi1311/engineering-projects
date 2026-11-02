"""Closed-loop visual teach-and-repeat, with arrival aligned to target view zero."""

import cv2
import numpy as np
from teach_agent import VisualTeachIndex as TeachAgent
from vis_nav_sdk import Action
from visual_pose import relative_pose
from visual_route import shorten_route


class MyAgent(TeachAgent):
    def hud(self):
        if not hasattr(self, "phase"):
            return []
        return [f"visual waypoint {self.index}/{self.end}", f"controller: {self.phase}"]

    def setup(self, info):
        super().setup(info)
        self.K = info.camera.intrinsic_matrix
        self.route = shorten_route(self, self.K)
        self.end = len(self.route) - 1
        self.index = 0
        self.phase = "route"
        self.last_ref = None
        self.failed_pose = 0
        self.last_direction = Action.RIGHT
        self.final_streak = 0
        self.predicted = None
        self.predicted_age = 0
        self.previous_command = None
        self.affine_cache = {}
        self.affine_detector = cv2.SIFT_create(nfeatures=1200)

    def affine_pose(self, ref, image):
        key = self.current if self.phase == "route" else "target0"
        if key not in self.affine_cache:
            variants = []
            for scale in (1.0, 1.5, 2.0, 0.75, 0.5, 3.0):
                warped = cv2.resize(ref, None, fx=scale, fy=1.0)
                kp, des = self.affine_detector.detectAndCompute(warped, None)
                points = np.float32([[k.pt[0] / scale, k.pt[1]] for k in kp])
                variants.append((points, des))
            self.affine_cache[key] = variants
        kp, des = self.affine_detector.detectAndCompute(image, None)
        live = (np.float32([k.pt for k in kp]), des)
        best = None
        for features in self.affine_cache[key]:
            candidate = relative_pose(features, live, ref, self.K, self.bf)
            if candidate is None or candidate[3] < 10:
                continue
            if np.hypot(candidate[0], candidate[1]) > 2.5 or abs(candidate[2]) > 0.9:
                continue
            if best is None or candidate[3] > best[3]:
                best = candidate
        return best

    def command(self, action, ticks):
        self.previous_command = (action, ticks)
        return action, ticks

    def recover_waypoint(self, image, live):
        start = max(0, self.index - 30) if self.failed_pose >= 60 else self.index + 1
        indices = list(range(start, min(self.end + 1, self.index + 31)))
        if not indices:
            return False
        nodes = [self.route[i] for i in indices]
        loss = np.mean(abs(self.thumbs[nodes] - self.thumb(image)), axis=(1, 2, 3))
        for rank in np.argsort(loss)[:8]:
            index = indices[int(rank)]
            if index == self.index:
                continue
            node = self.route[index]
            ref = cv2.imread(str(self.frames[node][0]))
            pose = relative_pose(self.features[node], live, ref, self.K, self.bf)
            if pose is None or pose[3] < 20:
                continue
            if np.hypot(pose[0], pose[1]) > 1.0 or abs(pose[2]) > 0.35:
                continue
            print("visual relocalization", self.index, "->", index, "inliers", pose[3], flush=True)
            self.index = index
            self.last_ref = None
            self.predicted = None
            self.failed_pose = 0
            return True
        return False

    def act(self, obs):
        if self.predicted is not None and self.previous_command is not None:
            action, ticks = self.previous_command
            x, z, yaw, n = self.predicted
            delta = (
                0.06 if action == Action.LEFT else -0.06 if action == Action.RIGHT else 0
            ) * ticks
            if delta:
                x, z = np.cos(delta) * x + np.sin(delta) * z, -np.sin(delta) * x + np.cos(delta) * z
                yaw += delta
            elif action in (Action.FORWARD, Action.BACKWARD):
                z -= (0.2 if action == Action.FORWARD else -0.2) * ticks
            self.predicted = (x, z, yaw, n)
            self.predicted_age += ticks
        self.previous_command = None
        kp, des = self.det.detectAndCompute(obs.image, None)
        live = (np.float32([k.pt for k in kp]), des)
        if self.phase == "rotate":
            if self.align_ticks > 0:
                n = min(4, self.align_ticks)
                self.align_ticks -= n
                return self.align_action, n
            self.phase = "final"
            self.failed_pose = 0
            self.predicted = None
            self.positioning = False
        if self.phase == "route":
            node = self.route[self.index]
            self.current = node
            if self.last_ref != node:
                self.ref_image = cv2.imread(str(self.frames[node][0]))
                self.last_ref = node
                self.failed_pose = 0
                self.predicted = None
                self.recovery_ticks = 0
                self.positioning = False
            feat = self.features[node]
            ref = self.ref_image
        else:
            feat = self.target_features[0]
            ref = self.targets[0]
        pose = relative_pose(feat, live, ref, self.K, self.bf)
        if pose is not None and pose[3] < 12:
            pose = None
        if (
            pose is None
            and self.phase == "route"
            and self.failed_pose % 10 == 0
            and self.recover_waypoint(obs.image, live)
        ):
            return Action.IDLE, 0
        affine_measured = False
        if pose is None and self.failed_pose % 8 == 0:
            pose = self.affine_pose(ref, obs.image)
            affine_measured = pose is not None
        measured = pose is not None
        if pose is None:
            self.failed_pose += 1
            if self.predicted is not None and self.predicted_age <= 80:
                pose = self.predicted
            else:
                if self.phase == "route" and self.recovery_ticks < 10:
                    # A capture label is an approach hint, not proof of arrival.
                    # Advance physically in small increments until vision recovers.
                    # Recorded actions belong to the SOURCE frame of an edge.
                    # The destination label may already be the next turn.
                    action = Action.IDLE
                    if self.index == 0:
                        names = self.frames[self.current][1]
                        reverse = False
                    else:
                        previous = self.route[self.index - 1]
                        same_route = (
                            self.frames[previous][0].parent == self.frames[self.current][0].parent
                        )
                        reverse = same_route and self.current == previous - 1
                        if same_route and self.current == previous + 1:
                            source = previous if self.recovery_ticks < 5 else self.current
                            names = self.frames[source][1]
                        elif reverse:
                            source = self.current
                            if self.recovery_ticks >= 5 and self.current > 0:
                                candidate = self.current - 1
                                if (
                                    self.frames[candidate][0].parent
                                    == self.frames[source][0].parent
                                ):
                                    source = candidate
                            names = self.frames[source][1]
                        else:
                            names = ["IDLE"]
                    if reverse:
                        # Reversing linear motion also reverses steering in the SDK.
                        moving = ("FORWARD" in names) != ("BACKWARD" in names)
                        inverse = {"FORWARD": "BACKWARD", "BACKWARD": "FORWARD"}
                        if not moving:
                            inverse.update({"LEFT": "RIGHT", "RIGHT": "LEFT"})
                        names = [inverse.get(name, name) for name in names]
                    for name in names:
                        action |= Action[name]
                    self.recovery_ticks += 1
                    if action != Action.IDLE:
                        print(
                            "visual approach",
                            obs.step,
                            "ref",
                            self.index,
                            "action",
                            action,
                            flush=True,
                        )
                        return self.command(action, 1)
                # Small heading probes retain visual overlap rather than asserting a node jump.
                # Scan through a complete heading cycle. Alternating every twelve
                # ticks only searched a 41-degree sector and could miss the view.
                print(
                    "pose unavailable",
                    obs.step,
                    "ref",
                    self.current,
                    "tries",
                    self.failed_pose,
                    flush=True,
                )
                return self.command(self.last_direction, 1)
        else:
            self.failed_pose = 0
            self.predicted = pose
            self.predicted_age = 0
        x, z, yaw, n = pose
        distance = float(np.hypot(x, z))
        print(
            "pose",
            obs.step,
            "ref",
            self.index,
            "phase",
            self.phase,
            "t",
            round(x, 3),
            round(z, 3),
            "yaw",
            round(yaw, 3),
            "inliers",
            n,
            flush=True,
        )
        tolerance = 0.25 if self.phase == "route" else 0.3
        if distance > tolerance:
            self.positioning = True
        if affine_measured and distance > 0.12:
            self.positioning = True
        if distance < 0.12:
            self.positioning = False
        if distance < tolerance and not self.positioning:
            if abs(yaw) > 0.065:
                return self.command(
                    Action.RIGHT if yaw > 0 else Action.LEFT, max(1, min(4, round(abs(yaw) / 0.06)))
                )
            if self.phase == "route" and measured:
                if self.index < self.end:
                    self.index += 1
                    # No movement or budget burned while changing the reference view.
                    return Action.IDLE, 0
                self.phase = "rotate"
                angle = self.goal_view * np.pi / 2
                if angle > np.pi:
                    angle -= 2 * np.pi
                self.align_action = Action.RIGHT if angle >= 0 else Action.LEFT
                self.align_ticks = round(abs(angle) / 0.06)
                return Action.IDLE, 0
            # The relative 3D pose measures position and heading independently.
            # A fixed pixel displacement is not a metric arrival test: nearby wall
            # features can shift many pixels even at a centimetre pose error.
            if self.phase == "final" and measured and n >= 20:
                self.final_streak += 1
                if self.final_streak >= 2:
                    return Action.CHECKIN
                return Action.IDLE, 1
        self.final_streak = 0
        # Drive toward the reference position, reversing when that requires less turn.
        forward = z >= 0
        bearing = np.arctan2(x, z) if forward else np.arctan2(-x, -z)
        # Do not turn toward millimetre lateral errors while the measured route
        # pose and heading are already reliable. Final alignment stays unchanged.
        if self.phase == "route" and measured and n >= 20 and abs(x) <= 0.06 and abs(yaw) <= 0.15:
            bearing = 0.0
        if abs(bearing) > 0.12:
            return self.command(
                Action.RIGHT if bearing > 0 else Action.LEFT,
                max(1, min(4, round(abs(bearing) / 0.06))),
            )
        return self.command(Action.FORWARD if forward else Action.BACKWARD, 1)

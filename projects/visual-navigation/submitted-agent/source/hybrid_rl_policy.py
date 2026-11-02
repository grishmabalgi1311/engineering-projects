"""Use target-relative geometry to avoid stopping at the recorded goal pose."""

import numpy as np
import cv2
from learned_goal import geometry
from hybrid_rl_learned import MyAgent as LocalAgent
from spatial_map import rotation, wrap
from vis_nav_sdk import Action
from visual_pose import relative_pose


class MyAgent(LocalAgent):
    def setup(self, info):
        super().setup(info)
        self.target_relative_pose = geometry(self)
        self.target_pose_finished = False
        points, des = cv2.SIFT_create(nfeatures=1600).detectAndCompute(self.targets[0], None)
        self.front_features = (np.float32([p.pt for p in points]), des)
        self.front_streak = 0
        self.front_step = None

    def act(self, obs):
        before = self.phase
        result = super().act(obs)
        if not self.target_pose_finished and before == "route" and self.index >= self.end - 40:
            points, des = self.local_detector.detectAndCompute(obs.image, None)
            live = (np.float32([p.pt for p in points]), des)
            front = relative_pose(self.front_features, live, self.targets[0], self.K, self.bf)
            if (
                front is not None
                and front[3] >= 20
                and np.hypot(front[0], front[1]) < 1.5
                and abs(front[2]) < 1.2
            ):
                if self.front_step != obs.step:
                    self.front_streak += 1
                    self.front_step = obs.step
            else:
                self.front_streak = 0
            if self.front_streak >= 2:
                self.target_pose_finished = True
                self.local_final = True
                self.phase = "final"
                self.det = self.local_detector
                self.target_features[0] = self.front_features
                self.predicted = None
                self.previous_command = None
                self.positioning = False
                print("verified front target acquired", obs.step, front[3], flush=True)
                return Action.IDLE, 0
        if (
            self.target_pose_finished
            or self.target_relative_pose is None
            or before != "route"
            or not self.local_final
            or self.current != self.goal
            or self.predicted is None
            or self.predicted_age > 12
        ):
            return result
        x, z, yaw, n = self.predicted
        if n < 10:
            return result
        goal = self.target_relative_pose
        offset = -rotation(-goal[2]) @ np.array(goal[:2])
        vector = np.array([x, z]) + rotation(yaw) @ offset
        error = wrap(yaw - goal[2] + wrap(self.goal_view * np.pi / 2))
        if np.linalg.norm(vector) < 0.18:
            self.target_pose_finished = True
            self.phase = "rotate"
            self.align_action = Action.RIGHT if error >= 0 else Action.LEFT
            self.align_ticks = round(abs(error) / 0.06)
            self.predicted = None
            self.previous_command = None
            print("target pose front alignment", obs.step, float(error), flush=True)
            return Action.IDLE, 0
        # Retain the reference image for localization, while steering toward
        # the actual target position inferred from its supplied image.
        self.phase = "route"
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
        return self.command(action, ticks)


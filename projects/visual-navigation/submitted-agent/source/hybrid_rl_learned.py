"""Learned geometric constraints with the independent local path controller."""

from learned_motion import augment
from hybrid_rl_local import MyAgent as LocalAgent
from spatial_map import rotation
import numpy as np


class MyAgent(LocalAgent):
    def setup(self, info):
        super().setup(info)
        self.spatial_map = augment(self, self.spatial_map)
        self.local_motion = {
            (a, b): (np.array([x, z]), angle, weight)
            for a, b, x, z, angle, weight in self.spatial_map["edges"]
        }
        for (a, b), (p, angle, weight) in list(self.local_motion.items()):
            self.local_motion.setdefault((b, a), (-rotation(-angle) @ p, -angle, weight))

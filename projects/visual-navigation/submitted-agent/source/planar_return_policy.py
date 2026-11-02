"""Experimental out-and-back route pruning; preserves the hybrid controller."""
from connected_word_policy import MyAgent as Working
from planar_return_route import shorten
from route_motion_connections import attach_connections
from learned_motion import augment
from spatial_map import rotation
import numpy as np


class MyAgent(Working):
    def setup(self,info):
        super().setup(info)
        route,accepted=shorten(self)
        self.route=route;self.end=len(route)-1
        self.spatial_map=augment(self,self.spatial_map)
        self.local_motion={(a,b):(np.array([x,z]),angle,weight)
                           for a,b,x,z,angle,weight in self.spatial_map['edges']}
        for (a,b),(p,angle,weight) in list(self.local_motion.items()):
            self.local_motion.setdefault((b,a),(-rotation(-angle)@p,-angle,weight))
        attach_connections(self)
        for (a,b),evidence in accepted.items():
            p=evidence['position'];angle=evidence['angle']
            self.local_motion[a,b]=(p,angle,1.)
            self.local_motion[b,a]=(-rotation(-angle)@p,-angle,1.)

"""Re-express a fresh visual pose along verified local motion edges."""
import numpy as np
from tick_stable_local import MyAgent as LocalAgent
from spatial_map import rotation, wrap


def reexpress(route, motion, start, end, pose):
    if end <= start or end-start > 40:
        return None
    position=np.zeros(2);angle=0.;travel=0.
    for index in range(start,end):
        edge=motion.get((route[index],route[index+1]))
        if edge is None or edge[2] < .9:
            return None
        travel+=float(np.linalg.norm(edge[0]))
        if travel>1.2:
            return None
        position+=rotation(angle)@edge[0]
        angle=wrap(angle+edge[1])
    translation=np.asarray(pose[:2])+rotation(pose[2])@position
    return (*translation,wrap(pose[2]+angle),pose[3])


class MyAgent(LocalAgent):
    def setup(self,info):
        super().setup(info)
        self.visual_anchor=None

    def localize(self,image,features):
        anchor=self.local_anchor
        if self.visual_anchor is not None:
            self.local_anchor=self.visual_anchor
        try:
            correction=super().localize(image,features)
        finally:
            self.local_anchor=anchor
        if correction is None:
            return None
        index,pose=correction
        self.visual_anchor=index
        # Only rebase an actually observed pose, and only along strong edges.
        if self.index-index>=8:
            transformed=reexpress(self.route,self.local_motion,index,self.index,pose)
            if transformed is not None:
                print("rebased visual anchor",index,"->",self.index,flush=True)
                return self.index,transformed
        return correction

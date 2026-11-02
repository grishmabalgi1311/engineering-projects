"""Transfer a fresh inferred pose across controller handoff, without marking it measured."""
import cv2
import numpy as np
from hybrid_rl_reanchor import MyAgent as LocalAgent
from reanchor_local import reexpress
from pose_agent import MyAgent as VisualAgent

class MyAgent(LocalAgent):
    def parent_mode(self,obs,index,reason,until=None):
        target=min(index,self.end)
        inferred=None
        if self.local_pose is not None and self.local_anchor is not None and self.local_lost==0:
            pose=(*self.local_pose,20)
            inferred=pose if target==self.local_anchor else reexpress(self.route,self.local_motion,self.local_anchor,target,pose)
        if inferred is None or np.linalg.norm(inferred[:2])>1.2:
            return super().parent_mode(obs,index,reason,until=until)
        print("local controller handoff",obs.step,target,reason,"using inferred pose",flush=True)
        self.index=target
        self.local_parent_until=target if until is None else min(until,self.end)
        self.last_ref=self.route[target]
        self.current=self.last_ref
        self.ref_image=cv2.imread(str(self.frames[self.current][0]))
        self.predicted=inferred
        # At most16 ticks of prediction; a fresh visual pose is still required to advance/check in.
        self.predicted_age=64
        self.previous_command=None
        self.failed_pose=0;self.recovery_ticks=0;self.positioning=False
        self.local_anchor=None;self.local_pose=None;self.local_previous=None
        return VisualAgent.act(self,obs)

"""Intermediate reference headings need not be final goal headings."""
import numpy as np
from odometry_recovery_policy import MyAgent as Working
from vis_nav_sdk import Action

class MyAgent(Working):
    def setup(self,info):
        super().setup(info)
        self.position_handoffs=0
        self.handoff_after=0

    def act(self,obs):
        result=super().act(obs)
        action=result[0] if isinstance(result,tuple) else result
        if (self.phase!='route' or self.local_final or self.index>=self.end-12
                or self.local_parent_until is None or self.predicted is None
                or self.predicted_age!=0 or obs.step<self.handoff_after
                or action not in (Action.LEFT,Action.RIGHT)
                or self.current!=self.route[self.index]):return result
        x,z,yaw,n=self.predicted
        edge=self.local_motion.get((self.route[self.index],self.route[self.index+1]))
        if (n<25 or np.hypot(x,z)>.18 or abs(yaw)>1.2
                or edge is None or edge[2]<.9):return result
        self.local_anchor=self.index;self.visual_anchor=self.index
        self.local_pose=np.array([x,z,yaw]);self.local_lost=0
        self.local_previous=None;self.local_command=None;self.local_parent_until=None
        self.previous_command=None;self.predicted=None
        self.local_last_index=self.index;self.local_progress_step=obs.step;self.local_stalled=0
        self.handoff_after=obs.step+20;self.position_handoffs+=1
        print('POSITION HANDOFF',obs.step,self.index,'yaw',round(yaw,3),flush=True)
        return Action.IDLE,0

    def finish(self,result):
        print('POSITION HANDOFF count',self.position_handoffs,flush=True)
        return super().finish(result)

"""Calibrate recovery translation from public consecutive camera frames."""
from collections import deque, Counter
import numpy as np
from hybrid_recovery_policy import MyAgent as Working
from planar_odometry import estimate
from spatial_map import rotation, wrap
from vis_nav_sdk import Action

class MyAgent(Working):
    def setup(self, info):
        super().setup(info)
        self.odom_samples=deque(maxlen=61)
        self.odom_previous=None
        self.odom_counts=Counter()

    def act(self, obs):
        kp,des=self.local_detector.detectAndCompute(obs.image,None)
        features=(np.float32([k.pt for k in kp]),des)
        if self.odom_previous is not None:
            step,image,oldfeatures,action=self.odom_previous
            ticks=obs.step-step
            if ticks>0 and action in (Action.FORWARD,Action.BACKWARD):
                self.odom_counts['eligible']+=1
                move=estimate(oldfeatures,features,image,obs.image,self.K,self.bf)
                back=estimate(features,oldfeatures,obs.image,image,self.K,self.bf)
                if move is not None and back is not None and min(move[3],back[3])>=30:
                    closure=rotation(back[2])@np.array(move[:2])+np.array(back[:2])
                    speed=-move[1]/ticks*(1 if action==Action.FORWARD else -1)
                    if (.08<speed<.25 and abs(move[0])<.1 and abs(move[2])<.05
                            and abs(wrap(move[2]+back[2]))<.03 and np.linalg.norm(closure)<.06):
                        self.odom_samples.append(speed)
                        self.odom_counts['accepted']+=1
        if self.predicted is not None and self.previous_command is not None:
            action,ticks=self.previous_command
            x,z,yaw,n=self.predicted
            if action in (Action.LEFT,Action.RIGHT):
                delta=(.06 if action==Action.LEFT else -.06)*ticks
                yaw=wrap(yaw+delta)-delta
            elif action in (Action.FORWARD,Action.BACKWARD) and len(self.odom_samples)>=5:
                z+=(.2-float(np.median(self.odom_samples)))*ticks*(1 if action==Action.FORWARD else -1)
                self.odom_counts['adjustments']+=1
            self.predicted=(x,z,yaw,n)
        result=super().act(obs)
        action=result[0] if isinstance(result,tuple) else result
        self.odom_previous=(obs.step,obs.image.copy(),features,action)
        return result

    def finish(self,result):
        print('ODOMETRY calibration',dict(self.odom_counts),'speed',float(np.median(self.odom_samples)) if self.odom_samples else None,flush=True)
        return super().finish(result)

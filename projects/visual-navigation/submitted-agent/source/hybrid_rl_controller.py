"""Small PPO movement policy. Inputs are visual waypoint offsets, not simulator poses."""
from pathlib import Path
import numpy as np
import torch
from torch import nn

# forward/backward, then steering sign: LEFT=+1, RIGHT=-1 in camera transform.
MOTION=np.array([[1,0],[-1,0],[0,1],[0,-1],[1,1],[1,-1],[-1,1],[-1,-1]],np.float32)
CHECKPOINT=Path(__file__).resolve().parents[1]/'models'/'hybrid_rl_v1.pt'

class Network(nn.Module):
    def __init__(self):
        super().__init__()
        self.body=nn.Sequential(nn.Linear(2,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh())
        self.actor=nn.Linear(64,8);self.critic=nn.Linear(64,1)
    def forward(self,x):
        y=self.body(x/2.)
        return self.actor(y),self.critic(y).squeeze(-1)

def transition(points,choices,speed=.15,turn=.06):
    move,steer=MOTION[np.asarray(choices)].T
    angle=steer*turn*np.where(move<0,-1,1)
    c,s=np.cos(angle),np.sin(angle)
    x,z=np.asarray(points).T
    return np.column_stack((c*x+s*z,-s*x+c*z-move*speed))

class Controller:
    def __init__(self,path=CHECKPOINT):
        self.net=Network()
        data=torch.load(path,map_location='cpu',weights_only=True)
        self.net.load_state_dict(data['state_dict']);self.net.eval()
    def choose(self,vector):
        with torch.inference_mode():
            logits,_=self.net(torch.as_tensor(vector,dtype=torch.float32).reshape(1,2))
        return int(logits.argmax(1).item())

def guarded_choice(vector,choice):
    """Reject movement away from the waypoint or turning away from its bearing."""
    vector=np.asarray(vector,dtype=float)
    if not np.isfinite(vector).all() or not 0.6<=np.linalg.norm(vector)<=2.:return False
    move,steer=MOTION[choice]
    if move and np.sign(move)!=np.sign(vector[1]):return False
    after=transition(vector[None,:],np.array([choice]))[0]
    direction=1 if vector[1]>=0 else -1
    before_angle=abs(np.arctan2(direction*vector[0],direction*vector[1]))
    after_angle=abs(np.arctan2(direction*after[0],direction*after[1]))
    if after_angle>before_angle+.04:return False
    if move and np.linalg.norm(after)>np.linalg.norm(vector)+.01:return False
    return True

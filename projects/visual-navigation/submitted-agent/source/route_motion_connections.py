"""Recheck and transfer graph connection geometry to a local motion map."""
import cv2,numpy as np
from visual_pose import relative_pose,image_pose_consistent
from spatial_map import rotation,wrap

def attach_connections(agent,max_distance=.25,max_angle=.35):
 verified={(a,b) for a,b,_,_ in agent.route_connections}
 verified|={(b,a) for a,b in list(verified)}
 additions=[]
 for a,b in zip(agent.route,agent.route[1:]):
  if (a,b) in agent.local_motion or (a,b) not in verified:continue
  reference=cv2.imread(str(agent.frames[a][0]));live=cv2.imread(str(agent.frames[b][0]))
  pose=relative_pose(agent.features[a],agent.features[b],reference,agent.K,agent.bf)
  reverse=relative_pose(agent.features[b],agent.features[a],live,agent.K,agent.bf)
  if pose is None or reverse is None or min(pose[3],reverse[3])<25:continue
  if np.hypot(*pose[:2])>max_distance or abs(pose[2])>max_angle:continue
  closure=rotation(reverse[2])@np.array(pose[:2])+np.array(reverse[:2])
  if np.linalg.norm(closure)>.08 or abs(wrap(pose[2]+reverse[2]))>.05:continue
  if not image_pose_consistent(reference,live,pose,agent.K):continue
  delta=-rotation(-pose[2])@np.array(pose[:2]);angle=-pose[2]
  agent.local_motion[a,b]=(delta,angle,1.)
  agent.local_motion[b,a]=(-rotation(-angle)@delta,-angle,1.)
  additions.append((a,b,float(delta[0]),float(delta[1]),float(angle),1.))
 agent.spatial_map=dict(agent.spatial_map,edges=[*agent.spatial_map['edges'],*additions])
 print('CONNECTED ROUTE measured missing edges',[(a,b) for a,b,*_ in additions],flush=True)

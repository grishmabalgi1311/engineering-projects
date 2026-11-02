"""Transfer newly verified visual graph links into the local movement map."""
import cv2,numpy as np
from word_route_policy import MyAgent as Working
from visual_pose import relative_pose,image_pose_consistent
from spatial_map import rotation,wrap
class MyAgent(Working):
 def setup(self,info):
  super().setup(info)
  verified={(a,b) for a,b,_,_ in self.route_connections}
  verified|={(b,a) for a,b in list(verified)}
  additions=[]
  for a,b in zip(self.route,self.route[1:]):
   if (a,b) in self.local_motion or (a,b) not in verified:continue
   reference=cv2.imread(str(self.frames[a][0]));live=cv2.imread(str(self.frames[b][0]))
   pose=relative_pose(self.features[a],self.features[b],reference,self.K,self.bf)
   reverse=relative_pose(self.features[b],self.features[a],live,self.K,self.bf)
   if pose is None or reverse is None or min(pose[3],reverse[3])<25:continue
   if np.hypot(*pose[:2])>.25 or abs(pose[2])>.35:continue
   closure=rotation(reverse[2])@np.array(pose[:2])+np.array(reverse[:2])
   if np.linalg.norm(closure)>.08 or abs(wrap(pose[2]+reverse[2]))>.05:continue
   if not image_pose_consistent(reference,live,pose,self.K):continue
   delta=-rotation(-pose[2])@np.array(pose[:2]);angle=-pose[2]
   self.local_motion[a,b]=(delta,angle,1.)
   self.local_motion[b,a]=(-rotation(-angle)@delta,-angle,1.)
   additions.append((a,b,float(delta[0]),float(delta[1]),float(angle),1.))
  self.spatial_map=dict(self.spatial_map,edges=[*self.spatial_map['edges'],*additions])
  print('CONNECTED ROUTE measured missing edges',[(a,b) for a,b,*_ in additions],flush=True)

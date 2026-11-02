"""Estimate target pose relative to its matched exploration view from RGB only."""
import hashlib
import pickle
from pathlib import Path
import cv2
import numpy as np
from spatial_map import rotation,wrap


def geometry(agent):
    reference=cv2.imread(str(agent.frames[agent.goal][0]))
    target=agent.targets[agent.goal_view]
    signature=hashlib.sha256(reference.tobytes()+target.tobytes()+agent.K.tobytes()
        +Path(__file__).read_bytes()+Path(__file__).with_name('learned_matching.py').read_bytes()).hexdigest()
    cache=agent.target_cache_path.parent/f'goal_geometry_{signature[:16]}.pkl'
    if cache.exists():
        return pickle.loads(cache.read_bytes())
    from learned_matching import Matcher
    model=Matcher(agent.target_cache_path.parent.parent/'learned_matcher',upscale=True)
    pose=model.pose(reference,target,agent.K)
    valid=pose is not None and pose[3]>=20 and np.hypot(pose[0],pose[1])<2
    if valid:
        a,b=model.last_points
        reverse=model.pose_from_points(target,b,a,agent.K)
        valid=(reverse is not None and reverse[3]>=15
               and abs(wrap(reverse[2]+pose[2]))<.05
               and np.linalg.norm(np.array(reverse[:2])+rotation(reverse[2])@pose[:2])<.12)
    result=pose if valid else None
    cache.write_bytes(pickle.dumps(result))
    print('target-relative geometry',result,flush=True)
    return result

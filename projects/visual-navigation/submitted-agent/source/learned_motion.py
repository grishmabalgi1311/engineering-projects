"""Repair weak public-image motion constraints using learned correspondences."""
import hashlib
import pickle
from pathlib import Path
import cv2
import numpy as np
from spatial_map import rotation
from spatial_map import wrap
from visual_pose import image_pose_error


def augment(agent,mapping):
    pairs={(a,b) for a,b in zip(agent.route,agent.route[1:])}
    pairs.update({(b,a) for a,b in list(pairs)})
    signature=hashlib.sha256(mapping['signature'].encode()+repr(sorted(pairs)).encode()+Path(__file__).read_bytes()
                             +Path(__file__).with_name('learned_matching.py').read_bytes()).hexdigest()
    cache=agent.target_cache_path.parent/f'learned_motion_{signature[:16]}.pkl'
    if cache.exists():
        return pickle.loads(cache.read_bytes())
    from learned_matching import Matcher
    model=Matcher(agent.target_cache_path.parent.parent/'learned_matcher',upscale=True)
    edges=list(mapping['edges'])
    updates=[]
    for index,(a,b,x,z,angle,weight) in enumerate(edges):
        if weight>=.9 or (a,b) not in pairs:
            continue
        reference=cv2.imread(str(agent.frames[a][0]))
        live=cv2.imread(str(agent.frames[b][0]))
        pose=model.pose(reference,live,agent.K)
        if (pose is None or pose[3]<15 or pose[3]<.45*model.last_stats['valid']
                or np.hypot(pose[0],pose[1])>2.5 or abs(pose[2])>1.3):
            continue
        check=image_pose_error(reference,live,pose,agent.K)
        if check is None or check[0]>.08 or check[1]>.20 or check[2]<2 or check[3]<.3:
            continue
        p,q=model.last_points
        reverse=model.pose_from_points(live,q,p,agent.K)
        if (reverse is None or reverse[3]<15 or reverse[3]<.45*model.last_stats['valid']
                or abs(wrap(reverse[2]+pose[2]))>.05
                or np.linalg.norm(np.array(reverse[:2])+rotation(reverse[2])@pose[:2])>.12):
            continue
        delta=-rotation(-pose[2])@np.array(pose[:2])
        edges[index]=(a,b,float(delta[0]),float(delta[1]),-pose[2],1.)
        updates.append((a,b,pose))
        print('learned motion verified',a,b,'inliers',pose[3],flush=True)
    value=dict(mapping,signature=signature,edges=edges,learned_updates=updates)
    cache.write_bytes(pickle.dumps(value))
    print('learned weak edges repaired',len(updates),flush=True)
    return value

"""Conservative offline checks and routing over independently proposed RGB links."""
from pathlib import Path
import argparse, heapq, json, math, pickle, sys
import cv2
import numpy as np
SOURCE=Path(__file__).resolve().parents[1]/'source'
sys.path.insert(0,str(SOURCE))
from spatial_map import rotation,wrap
from visual_pose import wall_points,image_pose_consistent
from visible_pose_check import visible_error
from planar_pnp import from_points,project
from functools import lru_cache

parser=argparse.ArgumentParser();parser.add_argument('--links',required=True)
parser.add_argument('--out',required=True)
parser.add_argument('--max-distance',type=float,default=.4)
parser.add_argument('--multiwall',action='store_true')
parser.add_argument('--data',required=True)
parser.add_argument('--cache',required=True)
parser.add_argument('--camera-json',required=True)
args=parser.parse_args()
cache=Path(args.cache);out=Path(args.out)
K=np.asarray(json.loads(Path(args.camera_json).read_text())['camera_K'],dtype=float)
if K.shape!=(3,3):raise ValueError('3x3 public SDK camera_K required')
frames=[];routes=[];trid=[]
for manifest in sorted(Path(args.data).glob('traj_*/data_info.json')):
    tr=[]
    for row in json.loads(manifest.read_text()):
        tr.append(len(frames));frames.append(manifest.parent/row['image']);trid.append(len(routes))
    routes.append(tr)
mapping=pickle.load(next(cache.glob('spatial_map_*.pkl')).open('rb'))
motion={}
def transform(p,angle):
    T=np.eye(3);T[:2,:2]=rotation(angle);T[:2,2]=p;return T
for a,b,x,z,angle,w in mapping['edges']:
    if w<.9:continue
    T=transform([x,z],angle);motion[a,b]=T;motion[b,a]=np.linalg.inv(T)
cv2.setNumThreads(1);cv2.setRNGSeed(7)
det=cv2.SIFT_create(nfeatures=1800);bf=cv2.BFMatcher()
@lru_cache(maxsize=256)
def image(n):return cv2.imread(str(frames[n]))
@lru_cache(maxsize=256)
def feature(n):
    kp,d=det.detectAndCompute(image(n),None);return np.float32([p.pt for p in kp]),d
def measure(a,b):
    p,d=feature(a);q,e=feature(b)
    if d is None or e is None or len(e)<2:return None
    good=[m for pair in bf.knnMatch(d,e,k=2) if len(pair)==2 for m,n in [pair] if m.distance<.7*n.distance]
    if len(good)<25:return None
    p=p[[m.queryIdx for m in good]];q=q[[m.trainIdx for m in good]]
    pose=from_points(image(a),p,q,K)
    if pose is None or pose[3]<25:return None
    xyz,valid=wall_points(image(a),p,K);valid &= xyz[:,2]<15
    xyz,p,q=xyz[valid],p[valid],q[valid]
    projected,depth=project(xyz,pose[:3],K);keep=(np.linalg.norm(projected-q,axis=1)<3)&(depth>.1)
    if keep.sum()<25:return None
    spans=[float(np.ptp(points[keep,0])/320) for points in (p,q)]
    if min(spans)<.45:return None
    horizontal=xyz[keep][:,(0,2)]
    singular=np.linalg.svd(horizontal-horizontal.mean(axis=0),compute_uv=False)
    multiwall=bool(singular[1]>.4 and singular[1]/max(singular[0],1e-8)>.17)
    return pose,spans,multiwall

def clear_transit(a,p):
    # A larger translation requires observed free floor along the connection.
    # Reconstruct wall distances in a narrow band around its bearing; an endpoint
    # behind the image or an occluding wall provides no clearance evidence.
    x,z=p
    if z<=.05:return False
    u=K[0,0]*x/z+K[0,2]
    if u<16 or u>303:return False
    pixels=np.c_[np.linspace(u-12,u+12,13),np.full(13,K[1,2])]
    points,valid=wall_points(image(a),pixels,K)
    return bool(valid.all() and np.min(points[:,2])>z+.25)

accepted=[];rejected=[]
for edge in json.loads(Path(args.links).read_text())['links']:
    a,b=edge['a'],edge['b']
    f=measure(a,b);r=measure(b,a)
    reason=None
    if f is None or r is None:reason='insufficient broadly distributed independent features'
    if reason is None:
        multiwall=f[2] and r[2]
        f,r=f[0],r[0]
        if np.hypot(*f[:2])>args.max_distance or abs(f[2])>1.6:reason='refined pose outside same-place bounds'
        elif np.linalg.norm(rotation(r[2])@np.array(f[:2])+r[:2])>.06 or abs(wrap(f[2]+r[2]))>.03:reason='independent cycle inconsistent'
        elif not image_pose_consistent(image(a),image(b),f,K):
            checks=[visible_error(image(a),image(b),f,K),visible_error(image(b),image(a),r,K)]
            if not (args.multiwall and multiwall and all(e is not None and e['mean']<.04 and e['worst']<.08
                    and e['bands']>=2 and e['coverage']>.30 for e in checks)):
                reason='full-view photometric disagreement'
        if reason is None and np.hypot(*f[:2])>.4:
            forward=-rotation(-f[2])@np.array(f[:2]);backward=-rotation(-r[2])@np.array(r[:2])
            if not (clear_transit(a,forward) or clear_transit(b,backward)):
                reason='translation has no visible free-floor support'
    support=[]
    if reason is None:
        delta=-rotation(-f[2])@np.array(f[:2]);T=transform(delta,-f[2])
        for sign in (-1,1):
            aa,bb=a+sign,b+sign
            if not 0<=aa<len(frames) or not 0<=bb<len(frames):continue
            if trid[aa]!=trid[a] or trid[bb]!=trid[b]:continue
            if (a,aa) not in motion or (b,bb) not in motion:continue
            alt=measure(aa,bb)
            if alt is None:continue
            pose=alt[0];altT=transform(-rotation(-pose[2])@np.array(pose[:2]),-pose[2])
            expected=np.linalg.inv(motion[a,aa])@T@motion[b,bb]
            err=np.linalg.inv(expected)@altT
            if np.linalg.norm(err[:2,2])<.12 and abs(np.arctan2(err[0,1],err[0,0]))<.06:
                support.append([aa,bb])
        if not support:reason='no independent neighboring-frame support'
    if reason:
        rejected.append(dict(a=a,b=b,reason=reason));continue
    accepted.append(dict(a=a,b=b,position=delta.tolist(),angle=-f[2],inliers=[f[3],r[3]],support=support))
print('Strict validation accepted',len(accepted),'rejected',len(rejected),flush=True)
out.parent.mkdir(parents=True,exist_ok=True)
out.write_text(json.dumps(dict(accepted=accepted,rejected=rejected,scope='Image connections only; no target or route selected'),indent=2))

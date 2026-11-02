"""Offline place-connection experiment from RGB only. Never calls SDK run/connect."""
from pathlib import Path
from functools import lru_cache
import argparse, hashlib, heapq, json, pickle, sys, time, math
import cv2
import numpy as np
from sklearn.cluster import MiniBatchKMeans
from sklearn.neighbors import NearestNeighbors
from threadpoolctl import threadpool_limits

SOURCE=Path(__file__).resolve().parents[1]/'source'
sys.path.insert(0,str(SOURCE))
from planar_pnp import from_points
from visible_pose_check import visible_error
from spatial_map import rotation,wrap

parser=argparse.ArgumentParser()
parser.add_argument('--data',required=True)
parser.add_argument('--cache',required=True)
parser.add_argument('--out',required=True)
parser.add_argument('--camera-json',required=True,help='Public SDK camera metadata containing camera_K')
parser.add_argument('--stride',type=int,default=8)
parser.add_argument('--max-distance',type=float,default=.4)
args=parser.parse_args();data=Path(args.data);cache=Path(args.cache);out=Path(args.out);out.mkdir(exist_ok=True)
cv2.setNumThreads(1);cv2.setRNGSeed(7)
K=np.asarray(json.loads(Path(args.camera_json).read_text())['camera_K'],dtype=float)
if K.shape!=(3,3):raise ValueError('3x3 public SDK camera_K required')
frames=[];routes=[];trajectory=[]
for manifest in sorted(data.glob('traj_*/data_info.json')):
    tr=[]
    for item in json.loads(manifest.read_text()):
        tr.append(len(frames));frames.append(manifest.parent/item['image']);trajectory.append(len(routes))
    routes.append(tr)
features_file=cache/'teach_features.pkl'
with features_file.open('rb') as f:
    signature,features,thumbs=pickle.load(f)
del thumbs
keyframes=np.array([n for tr in routes for n in tr[::args.stride]],int)
retrieval_file=out/'diverse_retrieval.pkl'
if retrieval_file.exists():
    saved_signature,pairs=pickle.loads(retrieval_file.read_bytes())
    if saved_signature!=signature:raise RuntimeError('Retrieval dataset signature mismatch')
else:
    rng=np.random.default_rng(7);samples=[]
    for n in keyframes:
        d=features[n][1]
        if d is not None:samples.append(d[rng.choice(len(d),min(16,len(d)),replace=False)])
    with threadpool_limits(limits=1):
        model=MiniBatchKMeans(n_clusters=128,random_state=7,batch_size=2048,n_init=3,max_iter=80).fit(np.vstack(samples))
        hist=np.zeros((len(keyframes),128),np.float32)
        for i,n in enumerate(keyframes):
            d=features[n][1]
            if d is not None:hist[i]=np.bincount(model.predict(d),minlength=128)
        hist*=np.log((1+len(hist))/(1+(hist>0).sum(axis=0)))
        hist=np.sqrt(hist/(hist.sum(axis=1,keepdims=True)+1e-8))
        neighbors=NearestNeighbors(n_neighbors=min(160,len(hist)),metric='cosine',algorithm='brute').fit(hist).kneighbors(hist,return_distance=False)
    pairs=set()
    for i,a in enumerate(keyframes):
        chosen=[]
        for j in neighbors[i]:
            b=int(keyframes[j]);a=int(a)
            if trajectory[a]==trajectory[b] and abs(a-b)<100:continue
            if any(trajectory[b]==trajectory[c] and abs(b-c)<40 for c in chosen):continue
            chosen.append(b);pairs.add((min(a,b),max(a,b)))
            if len(chosen)>=20:break
    pairs=sorted(pairs)
    # Image-derived global coordinates propose additional pairs only. All must
    # pass the same independent image geometry and photometric verification.
    from scipy.spatial import cKDTree
    map_files=list(cache.glob('spatial_map_*.pkl'))
    if len(map_files)==1:
        mapping=pickle.load(map_files[0].open('rb'))
        pose_values=mapping['poses']
        coords=pose_values[keyframes,:2]
        _,near=cKDTree(coords).query(coords,k=min(80,len(coords)))
        additions=set()
        for i,a in enumerate(keyframes):
            chosen=[]
            for j in near[i]:
                b=int(keyframes[j]);a=int(a)
                if trajectory[a]==trajectory[b] and abs(a-b)<100:continue
                if any(trajectory[b]==trajectory[c] and abs(b-c)<20 for c in chosen):continue
                chosen.append(b);additions.add((min(a,b),max(a,b)))
                if len(chosen)>=12:break
        pairs=sorted(set(pairs)|additions)
    retrieval_file.write_bytes(pickle.dumps((signature,pairs)))
print('Diverse keyframes',len(keyframes),'candidate pairs',len(pairs),flush=True)
matcher=cv2.BFMatcher()
@lru_cache(maxsize=256)
def image(node):return cv2.imread(str(frames[node]))

def points(a,b):
    p,d=features[a];q,e=features[b]
    if d is None or e is None or len(e)<2:return None
    good=[m for pair in matcher.knnMatch(d,e,k=2) if len(pair)==2 for m,n in [pair] if m.distance<.7*n.distance]
    if len(good)<30:return None
    return p[[m.queryIdx for m in good]],q[[m.trainIdx for m in good]]

def verify(a,b):
    matches=points(a,b)
    if matches is None:return None
    p,q=matches
    # A 2-D match alone is insufficient. Fit an upright camera pose, independently
    # reverse it, and compare actual textured pixels in both visible overlaps.
    f=from_points(image(a),p,q,K)
    if f is None or f[3]<25 or np.linalg.norm(f[:2])>args.max_distance or abs(f[2])>1.6:return None
    other=points(b,a)
    if other is None:return None
    r=from_points(image(b),*other,K)
    if r is None or r[3]<25:return None
    cycle=float(np.linalg.norm(rotation(r[2])@np.array(f[:2])+r[:2]))
    if cycle>.08 or abs(wrap(f[2]+r[2]))>.04:return None
    checks=[visible_error(image(a),image(b),f,K),visible_error(image(b),image(a),r,K)]
    if any(e is None or e['mean']>.05 or e['worst']>.10 or e['bands']<2 or e['coverage']<.20 for e in checks):return None
    delta=-rotation(-f[2])@np.array(f[:2])
    return dict(a=a,b=b,position=delta.tolist(),angle=-f[2],inliers=[f[3],r[3]],
        cycle=cycle,photometric=checks)

saved=out/'diverse_verified_links.json'
if saved.exists():
    progress=json.loads(saved.read_text())
    if progress['signature']!=signature:raise RuntimeError('Verification dataset signature mismatch')
else:progress=dict(signature=signature,completed=0,links=[])
started=time.monotonic()
for i in range(progress['completed'],len(pairs)):
    a,b=pairs[i]
    evidence=verify(a,b)
    if evidence is not None:progress['links'].append(evidence)
    progress['completed']=i+1
    if (i+1)%250==0:
        saved.write_text(json.dumps(progress,indent=2))
        print('Verified pairs',i+1,'/',len(pairs),'accepted',len(progress['links']),
              'seconds',round(time.monotonic()-started,1),flush=True)
saved.write_text(json.dumps(progress,indent=2))
print('FINISHED',len(progress['links']),'image-verified connections',flush=True)

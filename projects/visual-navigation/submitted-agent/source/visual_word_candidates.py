"""Classical SIFT visual-word retrieval from public exploration descriptors."""
import hashlib,pickle
from pathlib import Path
import numpy as np
from sklearn.cluster import MiniBatchKMeans
from sklearn.neighbors import NearestNeighbors
from threadpoolctl import threadpool_limits

def candidates(agent):
 signature=hashlib.sha256(repr(agent.target_cache_signature).encode()+Path(__file__).read_bytes()).hexdigest()
 path=agent.target_cache_path.parent/f'visual_words_{signature[:16]}.pkl'
 if path.exists():return pickle.loads(path.read_bytes())
 rng=np.random.default_rng(7)
 samples=[]
 for _,d in agent.features:
  if d is not None and len(d):samples.append(d[rng.choice(len(d),min(8,len(d)),replace=False)])
 sample=np.vstack(samples).astype(np.float32)
 with threadpool_limits(limits=1):
  model=MiniBatchKMeans(n_clusters=64,random_state=7,batch_size=2048,n_init=3,max_iter=80).fit(sample)
  hist=np.zeros((len(agent.frames),64),np.float32)
  for i,(_,d) in enumerate(agent.features):
   if d is not None and len(d):hist[i]=np.bincount(model.predict(d.astype(np.float32)),minlength=64)
  hist*=np.log((1+len(hist))/(1+(hist>0).sum(axis=0)))
  hist=np.sqrt(hist/(hist.sum(axis=1,keepdims=True)+1e-8))
  nn=NearestNeighbors(n_neighbors=min(64,len(hist)),metric='cosine',algorithm='brute').fit(hist)
  result=nn.kneighbors(hist,return_distance=False)
 path.write_bytes(pickle.dumps(result))
 print('visual words candidates indexed',len(hist),flush=True)
 return result

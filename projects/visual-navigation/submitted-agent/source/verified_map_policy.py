"""Optional image-verified map links; cached geometry never contains goal truth."""
import hashlib,json
import numpy as np
from planar_return_policy import MyAgent as Working
from planar_return_route import cost
from visual_return_route import search
from spatial_map import rotation

class MyAgent(Working):
    def setup(self,info):
        super().setup(info)
        path=self.target_cache_path.parent/'image_verified_links.json'
        if not path.exists():
            print('VERIFIED MAP: no matching precomputed image links; keep current route',flush=True)
            return
        saved=json.loads(path.read_text())
        signature=hashlib.sha256(b''.join(p.read_bytes() for p in sorted(self.data_dir.glob('traj_*/data_info.json')))).hexdigest()
        if saved.get('manifest_sha256')!=signature or not np.allclose(saved['camera_K'],self.K):
            raise ValueError('Image link cache belongs to different exploration data or camera')
        for node,digest in saved['image_hashes'].items():
            if hashlib.sha256(self.frames[int(node)][0].read_bytes()).hexdigest()!=digest:
                raise ValueError('Image link source image changed; rebuild the public-image cache')
        graph=[list(edges) for edges in self._route_graph[1]]
        # Preserve already verified return edges in the current route.
        for a,b in zip(self.route,self.route[1:]):
            edge=self.local_motion.get((a,b))
            if edge is not None and edge[2]>=.9 and not any(node==b for node,_ in graph[a]):
                weight=cost(edge[0],edge[1]);graph[a].append((b,weight));graph[b].append((a,weight))
        new_motion={}
        for e in saved['links']:
            a,b=e['a'],e['b'];p=np.asarray(e['position'],float);angle=float(e['angle'])
            if p.shape!=(2,) or not np.isfinite(p).all() or not np.isfinite(angle):
                raise ValueError('Invalid verified image geometry')
            weight=cost(p,angle);graph[a].append((b,weight));graph[b].append((a,weight))
            new_motion[a,b]=(p,angle,1.)
            new_motion[b,a]=(-rotation(-angle)@p,-angle,1.)
        route,new_cost=search(graph,[r[0] for r in self.routes if r],self.goal)
        old_cost=sum(min(w for node,w in graph[a] if node==b) for a,b in zip(self.route,self.route[1:]))
        # A near-tie can merely switch to less recognizable views of the same
        # corridor, introducing recovery cost with no meaningful route saving.
        if new_cost>=.95*old_cost:
            print('VERIFIED MAP keep route: marginal saving',round(old_cost-new_cost,3),flush=True)
            return
        for a,b in zip(route,route[1:]):
            if (a,b) in new_motion and ((a,b) not in self.local_motion or self.local_motion[a,b][2]<.9):
                self.local_motion[a,b]=new_motion[a,b]
                self.local_motion[b,a]=new_motion[b,a]
        print('VERIFIED MAP route',len(self.route),'->',len(route),'public image links',len(saved['links']),flush=True)
        self.route=route;self.end=len(route)-1

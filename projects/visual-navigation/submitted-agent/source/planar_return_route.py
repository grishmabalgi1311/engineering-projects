"""Refine coarse return proposals in RGB reprojection geometry before routing.

Recorded poses are never read. Forward and reverse camera estimates independently
integrate each candidate, so a nearby frame can replace a biased proposal endpoint.
"""
import hashlib
import pickle
from pathlib import Path
import numpy as np
from visible_return_route import Verifier as BaseVerifier
from visible_pose_check import visible_error
from planar_pnp import from_points
from spatial_map import rotation, wrap
from visual_return_route import proposals, search


class Verifier(BaseVerifier):
    def __init__(self, agent):
        super().__init__(agent)
        self.signature = hashlib.sha256(self.signature.encode()+Path(__file__).read_bytes()
            +Path(__file__).with_name('planar_pnp.py').read_bytes()).hexdigest()
        self.cache = agent.target_cache_path.parent/f'planar_return_motion_{self.signature[:16]}.pkl'
        self.measured = pickle.loads(self.cache.read_bytes()) if self.cache.exists() else {}

    def points(self, a, b):
        p, d = self.feature(a);q, e = self.feature(b)
        if d is None or e is None or len(e) < 2:
            return np.empty((0, 2)), np.empty((0, 2))
        good = [m for pair in self.agent.bf.knnMatch(d, e, k=2) if len(pair)==2
                for m,n in [pair] if m.distance < .7*n.distance]
        return p[[m.queryIdx for m in good]], q[[m.trainIdx for m in good]]

    def evidence(self, a, b, f, r):
        if not self.consistent(f, r):return False
        for first, second, pose in ((a,b,f),(b,a,r)):
            e = visible_error(self.images[first], self.images[second], pose, self.agent.K)
            if e is None or e['mean']>.05 or e['worst']>.1 or e['bands']<2 or e['coverage']<.2:
                return False
        return True

    def measure(self, a, b):
        if (a,b) in self.measured:return self.measured[a,b]
        p,q=self.points(a,b);u,v=self.points(b,a);K=self.agent.K
        f=from_points(self.images[a],p,q,K);r=from_points(self.images[b],u,v,K)
        method='planar_sift'
        if not self.evidence(a,b,f,r):
            if self.learned is None:
                from learned_matching import Matcher
                self.learned=Matcher(self.agent.target_cache_path.parent.parent/'learned_matcher',upscale=True)
            p,q,_=self.learned.points(self.images[a],self.images[b])
            u,v,_=self.learned.points(self.images[b],self.images[a])
            f=from_points(self.images[a],p,q,K);r=from_points(self.images[b],u,v,K)
            method='planar_learned'
        result=(f,r,method) if self.evidence(a,b,f,r) else None
        self.measured[a,b]=result
        self.cache.write_bytes(pickle.dumps(self.measured))
        return result

    def refine(self, proposal):
        chain=proposal[0]
        edges=[self.measure(a,b) for a,b in zip(chain,chain[1:])]
        accepted={}
        for start in range(len(chain)-20):
            position=np.zeros(2);angle=0.;travel=0.;measurements=[]
            for last in range(start+1,len(chain)):
                row=edges[last-1]
                if row is None:break
                f=row[0];delta=-rotation(-f[2])@np.array(f[:2])
                position+=rotation(angle)@delta;angle=wrap(angle-f[2]);travel+=np.linalg.norm(delta)
                measurements.append(row)
                if last-start<20 or travel<=2 or abs(angle)<=1.3 or np.linalg.norm(position)>.2:
                    continue
                backward=np.zeros(2);back_angle=0.
                for _,r,_ in reversed(measurements):
                    backward+=rotation(back_angle)@(-rotation(-r[2])@np.array(r[:2]))
                    back_angle=wrap(back_angle-r[2])
                closure=float(np.linalg.norm(position+rotation(angle)@backward))
                if np.linalg.norm(backward)>.2 or closure>.12 or abs(wrap(angle+back_angle))>.05:
                    continue
                accepted[chain[start],chain[last]]=dict(position=position.copy(),angle=angle,
                    travel=travel,closure=closure,reverse_position=backward,captures=last-start,
                    methods={name:sum(x[2]==name for x in measurements)
                             for name in ('planar_sift','planar_learned')})
        return accepted


def cost(point, angle):
    return max(.2,(np.linalg.norm(point)/.15+abs(angle)/.06)/5)


def shorten(agent):
    verifier=Verifier(agent);graph=[list(edges) for edges in agent._route_graph[1]]
    signature=hashlib.sha256(repr((verifier.signature,agent.goal,graph,agent.spatial_map['signature'])).encode()
        +Path(__file__).with_name('visual_return_route.py').read_bytes()).hexdigest()
    path=agent.target_cache_path.parent/f'planar_return_route_{signature[:16]}.pkl'
    if path.exists():
        route,accepted,processed=pickle.loads(path.read_bytes())
    else:
        coarse=proposals(agent);accepted={};processed=set()
        for _ in range(len(coarse)+1):
            trial=[list(edges) for edges in graph]
            for (a,b),(_,point,angle,_) in coarse.items():
                if (a,b) in processed:continue
                weight=cost(point,angle);trial[a].append((b,weight));trial[b].append((a,weight))
            for (a,b),e in accepted.items():
                weight=cost(e['position'],e['angle']);trial[a].append((b,weight));trial[b].append((a,weight))
            route,_=search(trial,[r[0] for r in agent.routes if r],agent.goal)
            pending=[]
            for a,b in zip(route,route[1:]):
                key=min(a,b),max(a,b)
                if key in coarse and key not in processed and key not in accepted:pending.append(key)
            if not pending:break
            # Replan after each verification. refine examines every subchain,
            # so nested proposals need not repeat the same image measurements.
            pair=pending[0]
            chain=coarse[pair][0]
            trajectory=next(r for r in agent.routes if chain[0] in r)
            lo=max(0,trajectory.index(chain[0])-24)
            hi=min(len(trajectory),trajectory.index(chain[-1])+25)
            expanded=trajectory[lo:hi]
            found=verifier.refine((expanded,*coarse[pair][1:]))
            accepted.update(found)
            covered=set(expanded)
            processed.update(key for key in coarse if key[0] in covered and key[1] in covered)
            print('PLANAR RETURN refined',pair,'verified alternatives',len(found),flush=True)
        path.write_bytes(pickle.dumps((route,accepted,processed)))
    used=[(a,b) for a,b in zip(route,route[1:]) if (min(a,b),max(a,b)) in accepted]
    print('PLANAR RETURN route',len(agent.route),'->',len(route),'used',used,'verified',len(accepted),flush=True)
    return route,accepted

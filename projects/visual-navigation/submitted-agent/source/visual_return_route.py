"""Prune recorded out-and-back detours using independently checked visual motion.

An apparent return from integrated odometry is only a proposal. Every recorded
edge must have an independent reverse image measurement and the whole chain
must close consistently before it can become a short graph connection.
"""
import hashlib,heapq,pickle
from pathlib import Path
import cv2,numpy as np
from planar_odometry import estimate
from spatial_map import rotation,wrap


def search(graph,starts,goal):
    distance={s:0. for s in starts};previous={s:None for s in starts}
    queue=[(0.,s) for s in starts];heapq.heapify(queue)
    while queue:
        cost,node=heapq.heappop(queue)
        if cost!=distance[node]:continue
        if node==goal:break
        for b,weight in graph[node]:
            value=cost+weight
            if value<distance.get(b,float('inf')):
                distance[b]=value;previous[b]=node;heapq.heappush(queue,(value,b))
    if goal not in previous:return None
    path=[];node=goal
    while node is not None:path.append(node);node=previous[node]
    return path[::-1],distance[goal]


def proposals(agent):
    # Propose returns by accumulated camera motion, rather than truncating at
    # 100 image captures. Dense recordings otherwise hide ordinary dead-end
    # visits behind the frame-count limit. These are hypotheses only; the
    # unchanged planar verifier must independently reconstruct every edge.
    from scipy.spatial import cKDTree
    found={}
    for route in agent.routes:
        count=len(route)
        positions=np.zeros((count,2));angles=np.zeros(count)
        travel=np.zeros(count);uncertain=np.zeros(count,dtype=int)
        for i,(a,b) in enumerate(zip(route,route[1:]),1):
            edge=agent.local_motion.get((a,b))
            if edge is None:
                p,theta,weight=np.zeros(2),0.,0.
            else:
                p,theta,weight=edge
            positions[i]=positions[i-1]+rotation(angles[i-1])@p
            angles[i]=angles[i-1]+theta
            travel[i]=travel[i-1]+np.linalg.norm(p)
            uncertain[i]=uncertain[i-1]+int(weight<.9)
        nearby=cKDTree(positions).query_ball_point(positions,.25)
        for i,neighbors in enumerate(nearby):
            for j in neighbors:
                if j-i<20 or j-i>1000 or uncertain[j]!=uncertain[i]:continue
                length=float(travel[j]-travel[i])
                if not 2.<length<30.:continue
                angle=float(wrap(angles[j]-angles[i]))
                if abs(angle)<=1.3:continue
                position=rotation(-angles[i])@(positions[j]-positions[i])
                found[route[i],route[j]]=(list(route[i:j+1]),position,angle,length)
    print('MOTION-BOUNDED return proposals',len(found),flush=True)
    return found


def verify(agent,proposal,measured):
    chain,position,angle,travel=proposal
    back_position=np.zeros(2);back_angle=0.;error_sum=0.
    for a,b in zip(chain[-2::-1],chain[:0:-1]):
        edge=agent.local_motion[a,b];point,theta,_=edge
        if (b,a) not in measured:
            first=cv2.imread(str(agent.frames[b][0]));second=cv2.imread(str(agent.frames[a][0]))
            measured[b,a]=estimate(agent.features[b],agent.features[a],first,second,agent.K,agent.bf)
        reverse=measured[b,a]
        if reverse is None or reverse[3]<25:return None
        cycle=float(np.linalg.norm(np.array(reverse[:2])-point))
        if cycle>.06 or abs(wrap(reverse[2]-theta))>.03:return None
        error_sum+=cycle
        if error_sum>.35:return None
        delta=-rotation(-reverse[2])@np.array(reverse[:2])
        back_position+=rotation(back_angle)@delta;back_angle=wrap(back_angle-reverse[2])
    closure=float(np.linalg.norm(position+rotation(angle)@back_position))
    if np.linalg.norm(back_position)>.2 or closure>.12 or abs(wrap(angle+back_angle))>.05:
        return None
    return dict(position=position,angle=angle,travel=travel,closure=closure,error_sum=error_sum,captures=len(chain)-1)


def shorten(agent):
    graph=[list(edges) for edges in agent._route_graph[1]]
    signature=hashlib.sha256(repr((agent.target_cache_signature,agent.goal,agent.K.tolist(),
       agent.spatial_map['signature'],agent._route_graph[1])).encode()+Path(__file__).read_bytes()
       +Path(__file__).with_name('planar_odometry.py').read_bytes()).hexdigest()
    cache=agent.target_cache_path.parent/f'visual_return_{signature[:16]}.pkl'
    if cache.exists():
        route,accepted,rejected=pickle.loads(cache.read_bytes())
    else:
        proposed=proposals(agent);accepted={};rejected=[];measured={}
        starts=[r[0] for r in agent.routes if r]
        # Verify only proposals that can change the selected route. A rejected
        # proposal is removed and search repeats; it never enters the controller.
        for _ in range(len(proposed)+1):
            trial=[list(e) for e in graph]
            for (a,b),(_,p,angle,_) in proposed.items():
                if (a,b) in rejected:continue
                cost=max(.2,(np.linalg.norm(p)/.15+abs(angle)/.06)/5)
                trial[a].append((b,cost));trial[b].append((a,cost))
            route,_=search(trial,starts,agent.goal)
            untested=[]
            for a,b in zip(route,route[1:]):
                key=(min(a,b),max(a,b))
                if key in proposed and key not in accepted and key not in rejected:untested.append(key)
            if not untested:break
            for pair in untested:
                evidence=verify(agent,proposed[pair],measured)
                if evidence is None:rejected.append(pair)
                else:accepted[pair]=evidence
                print('VISUAL RETURN candidate',pair,'accepted',evidence is not None,flush=True)
        cache.write_bytes(pickle.dumps((route,accepted,rejected)))
    print('VISUAL RETURN route',len(agent.route),'->',len(route),
          'accepted',len(accepted),'rejected',len(rejected),flush=True)
    return route,accepted

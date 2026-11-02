"""Package accepted image geometry, with source hashes, never a fixed goal/path."""
from pathlib import Path
import argparse,hashlib,json
p=argparse.ArgumentParser();p.add_argument('--data',required=True);p.add_argument('--verified',nargs='+',required=True);p.add_argument('--out',required=True);p.add_argument('--camera-json',required=True)
a=p.parse_args();data=Path(a.data);frames=[];manifests=sorted(data.glob('traj_*/data_info.json'))
for m in manifests:frames.extend(m.parent/r['image'] for r in json.loads(m.read_text()))
links={}
support_nodes=set()
for name in a.verified:
    for e in json.loads(Path(name).read_text())['accepted']:
        links[e['a'],e['b']]={k:e[k] for k in ('a','b','position','angle')}
        support_nodes.update(n for pair in e.get('support',[]) for n in pair)
nodes={n for pair in links for n in pair}|support_nodes
record=dict(format_version=1,manifest_sha256=hashlib.sha256(b''.join(m.read_bytes() for m in manifests)).hexdigest(),
    camera_K=json.loads(Path(a.camera_json).read_text())['camera_K'],
    image_hashes={str(n):hashlib.sha256(frames[n].read_bytes()).hexdigest() for n in sorted(nodes)},
    links=list(links.values()),provenance='Independent RGB geometry, cycle, image-overlap, neighboring-frame and visible-floor checks. No goal or evaluated robot pose.')
out=Path(a.out);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(record,indent=2))
print('Exported',len(links),'public-image connections; no route or goal stored.')

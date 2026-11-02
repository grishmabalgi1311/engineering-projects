"""Build public-image feature, graph and odometry caches without a session."""
from pathlib import Path
import argparse,json,sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'source'))
from teach_agent import VisualTeachIndex
from word_visual_route import shorten_route
from spatial_map import build_map
p=argparse.ArgumentParser(__doc__)
p.add_argument('--data',required=True);p.add_argument('--cache',required=True)
p.add_argument('--camera-json',required=True)
a=p.parse_args();K=np.asarray(json.loads(Path(a.camera_json).read_text())['camera_K'],dtype=float)
if K.shape!=(3,3):raise ValueError('3x3 public SDK camera_K required')
agent=VisualTeachIndex(Path(a.data),cache_dir=Path(a.cache))
# The helper builds the full graph. Its temporary terminal is simply the last
# public frame; it is not a target, a robot pose or a fixed navigation route.
agent.goal=len(agent.frames)-1;agent.end=len(agent.frames)-1
shorten_route(agent,K);build_map(agent,K)
print('Public RGB caches prepared. No session, goal selection or score.')

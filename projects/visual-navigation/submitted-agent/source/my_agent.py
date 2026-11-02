"""Portable entry point for the preserved verified 7/10 agent.

All 37 policy/support modules are byte-identical to the recorded source.
This file changes only SDK launch arguments and external resource locations.
"""
from pathlib import Path
import os
import sys
import cli
import hybrid_rl_controller
import learned_matching

ROOT=Path(__file__).resolve().parents[1]
_OriginalController=hybrid_rl_controller.Controller
class ExternalCheckpointController(_OriginalController):
    def __init__(self,path=None):
        resolved=path or os.environ.get('VIS_NAV_PPO_CHECKPOINT') or ROOT/'models/hybrid_rl_v1.pt'
        super().__init__(Path(resolved))
hybrid_rl_controller.Controller=ExternalCheckpointController
_OriginalMatcher=learned_matching.Matcher
class ExternalMatcher(_OriginalMatcher):
    def __init__(self,cache,upscale=False):
        super().__init__(Path(os.environ.get('VIS_NAV_MATCHER_CACHE',str(cache))),upscale=upscale)
learned_matching.Matcher=ExternalMatcher
from planar_return_agent import MyAgent

def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    p=cli.parser(__doc__)
    p.add_argument('--data',help='Existing exploration directory, otherwise SDK download/recording')
    p.add_argument('--cache-dir',help='Optional directory of public-image-derived caches')
    p.add_argument('--ppo-checkpoint',help='External trained PPO checkpoint; excluded from submission')
    p.add_argument('--matcher-cache',help='External directory containing the official LightGlue weights')
    p.add_argument('--headless',action='store_true')
    p.add_argument('--motion-seed',type=int)
    p.add_argument('--max-steps',type=int)
    a=p.parse_args()
    if a.local is None and (a.motion_seed is not None or a.max_steps is not None):
        p.error('--motion-seed and --max-steps are local-only')
    if a.ppo_checkpoint:os.environ['VIS_NAV_PPO_CHECKPOINT']=str(Path(a.ppo_checkpoint).resolve())
    if a.matcher_cache:os.environ['VIS_NAV_MATCHER_CACHE']=str(Path(a.matcher_cache).resolve())
    checkpoint=Path(os.environ.get('VIS_NAV_PPO_CHECKPOINT',ROOT/'models/hybrid_rl_v1.pt'))
    if not checkpoint.is_file():
        p.error('The required PPO checkpoint is external. Supply --ppo-checkpoint PATH; see MODEL_SETUP.md.')
    challenge=cli.challenge(a)
    cache=Path(a.cache_dir) if a.cache_dir else ROOT/'cache'/challenge.replace(':','-')
    agent=MyAgent(cli.exploration_data(a,a.data),cache_dir=cache)
    options=cli.run_options(a);options['viewer']=not a.headless
    if a.motion_seed is not None:options['motion_seed']=a.motion_seed
    if a.max_steps is not None:options['max_steps']=a.max_steps
    from vis_nav_sdk import run
    run(agent,challenge,**options)

if __name__=='__main__':main()

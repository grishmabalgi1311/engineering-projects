# External model resources

Weights are intentionally excluded from this submission, as requested by the
course form. They remain required runtime inputs.

1. Custom PPO waypoint controller: hybrid_rl_v1.pt.
   SHA256: 206c3165fe015ec2582374a73e26b43fe3b464de4500907c4a6184279ef1e6b1
   Supply --ppo-checkpoint PATH or VIS_NAV_PPO_CHECKPOINT. The existing project
   copy is in vis_nav_player/models/. This package does not claim that retraining
   an unspecified model would exactly reproduce this checkpoint.

2. Official pretrained SuperPoint/LightGlue matching model:
   ETH-CVG/lightglue_superpoint through Hugging Face Transformers.
   Supply --matcher-cache PATH or VIS_NAV_MATCHER_CACHE. The existing project
   cache is vis_nav_player/cache/learned_matcher/. Normal agent execution uses
   local_files_only=True. On a new machine an authorized model setup step must
   populate that cache before the agent runs.

The original controller and matcher source are included. Do not encode weights
inside source or add model files to the upload to work around the exclusion.

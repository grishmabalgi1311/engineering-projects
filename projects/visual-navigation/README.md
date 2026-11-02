# Camera-only navigation

Visual place recognition and topological navigation toward image-defined goals.

**Tools:** Python / Visual recognition / Dijkstra / Visual odometry / Guarded PPO waypoint proposals  
**Status:** Evaluated course implementation

## Contribution and documented behavior

- Extended a supplied camera-only navigation framework with image-verified graph links, visual odometry, recovery and final image alignment.
- The saved midterm comparison improved score from 6/10 to 7/10 while reducing executed ticks from 2,451 to 1,418 (42.1%).
- SPL increased from 0.358 to 0.619. The comparison does not isolate the causal contribution of each individual change.

## Files

- [evidence/midterm-comparison.json](evidence/midterm-comparison.json)
- [evidence/separate-course-run-718.json](evidence/separate-course-run-718.json)
- [reports/Engineering_Methods_7of10.pdf](reports/Engineering_Methods_7of10.pdf)
- [submitted-agent/.env.example](submitted-agent/.env.example)
- [submitted-agent/MODEL_SETUP.md](submitted-agent/MODEL_SETUP.md)
- [submitted-agent/SOURCE_PROVENANCE.json](submitted-agent/SOURCE_PROVENANCE.json)
- [submitted-agent/TESTED_DEPENDENCIES.json](submitted-agent/TESTED_DEPENDENCIES.json)
- [submitted-agent/pyproject.toml](submitted-agent/pyproject.toml)
- [submitted-agent/source/cli.py](submitted-agent/source/cli.py)
- [submitted-agent/source/connected_word_policy.py](submitted-agent/source/connected_word_policy.py)
- [submitted-agent/source/hybrid_recovery_policy.py](submitted-agent/source/hybrid_recovery_policy.py)
- [submitted-agent/source/hybrid_rl_controller.py](submitted-agent/source/hybrid_rl_controller.py)
- [submitted-agent/source/hybrid_rl_learned.py](submitted-agent/source/hybrid_rl_learned.py)
- [submitted-agent/source/hybrid_rl_local.py](submitted-agent/source/hybrid_rl_local.py)
- [submitted-agent/source/hybrid_rl_local_base.py](submitted-agent/source/hybrid_rl_local_base.py)
- [submitted-agent/source/hybrid_rl_policy.py](submitted-agent/source/hybrid_rl_policy.py)
- [submitted-agent/source/hybrid_rl_reanchor.py](submitted-agent/source/hybrid_rl_reanchor.py)
- [submitted-agent/source/learned_goal.py](submitted-agent/source/learned_goal.py)
- [submitted-agent/source/learned_matching.py](submitted-agent/source/learned_matching.py)
- [submitted-agent/source/learned_motion.py](submitted-agent/source/learned_motion.py)
- [submitted-agent/source/my_agent.py](submitted-agent/source/my_agent.py)
- [submitted-agent/source/odometry_recovery_policy.py](submitted-agent/source/odometry_recovery_policy.py)
- [submitted-agent/source/oscillation_recovery.py](submitted-agent/source/oscillation_recovery.py)
- [submitted-agent/source/planar_odometry.py](submitted-agent/source/planar_odometry.py)
- [submitted-agent/source/planar_pnp.py](submitted-agent/source/planar_pnp.py)
- [submitted-agent/source/planar_return_agent.py](submitted-agent/source/planar_return_agent.py)
- [submitted-agent/source/planar_return_policy.py](submitted-agent/source/planar_return_policy.py)
- [submitted-agent/source/planar_return_route.py](submitted-agent/source/planar_return_route.py)
- [submitted-agent/source/pose_agent.py](submitted-agent/source/pose_agent.py)
- [submitted-agent/source/position_handoff_policy.py](submitted-agent/source/position_handoff_policy.py)
- [submitted-agent/source/reanchor_local.py](submitted-agent/source/reanchor_local.py)
- [submitted-agent/source/route_motion_connections.py](submitted-agent/source/route_motion_connections.py)
- [submitted-agent/source/run_diagnostics.py](submitted-agent/source/run_diagnostics.py)
- [submitted-agent/source/spatial_map.py](submitted-agent/source/spatial_map.py)
- [submitted-agent/source/teach_agent.py](submitted-agent/source/teach_agent.py)
- [submitted-agent/source/tick_stable_local.py](submitted-agent/source/tick_stable_local.py)
- [submitted-agent/source/verified_map_policy.py](submitted-agent/source/verified_map_policy.py)
- [submitted-agent/source/visible_pose_check.py](submitted-agent/source/visible_pose_check.py)
- [submitted-agent/source/visible_return_route.py](submitted-agent/source/visible_return_route.py)
- [submitted-agent/source/visual_pose.py](submitted-agent/source/visual_pose.py)
- [submitted-agent/source/visual_return_route.py](submitted-agent/source/visual_return_route.py)
- [submitted-agent/source/visual_route.py](submitted-agent/source/visual_route.py)
- [submitted-agent/source/visual_word_candidates.py](submitted-agent/source/visual_word_candidates.py)
- [submitted-agent/source/wall_orientation.py](submitted-agent/source/wall_orientation.py)
- [submitted-agent/source/word_route_policy.py](submitted-agent/source/word_route_policy.py)
- [submitted-agent/source/word_visual_route.py](submitted-agent/source/word_visual_route.py)
- [submitted-agent/tools/discover_image_connections.py](submitted-agent/tools/discover_image_connections.py)
- [submitted-agent/tools/export_image_connections.py](submitted-agent/tools/export_image_connections.py)
- [submitted-agent/tools/prepare_public_images.py](submitted-agent/tools/prepare_public_images.py)
- [submitted-agent/tools/verify_image_connections.py](submitted-agent/tools/verify_image_connections.py)

## Setup and verification boundary

The submitted-agent/ package is the preserved 7/10 midterm implementation. Follow its MODEL_SETUP.md and pyproject.toml; course SDK access, exploration data, a PPO checkpoint and matching resources may be required. Excluded resources are not silently replaced. The separate 718-step JSON is a different saved evaluation and must not be mixed with the 2,451→1,418 midterm comparison. No new hidden-maze evaluation was run for this publication.

## Scope and attribution

Saved midterm run comparison; not a cross-environment success rate or a causal ablation.

Team/course projects include inherited frameworks and collaborators. File inclusion is not a claim of sole authorship. Source files retain their original comments and attribution. No blanket license is applied to third-party or jointly authored material.

## Next step

Extend evaluation across unseen environments and analyze localization and route-efficiency failures.

[Portfolio case study](https://grishmabalgi1311.github.io/grishma-portfolio/#project=visual-navigation) · [Grishma Balgi](https://github.com/grishmabalgi1311)

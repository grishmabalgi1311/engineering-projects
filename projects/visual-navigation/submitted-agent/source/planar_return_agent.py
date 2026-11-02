"""Experimental hybrid PPO waypoint controller.

Run: uv run python source/planar_return_agent.py --local 7
Only SDK images, calibration, and exploration data enter the policy.
Original graph agent: image_matching_audit/backups/my_agent_original.py.
"""

import sys
from pathlib import Path

import cli
from verified_map_policy import MyAgent as Controller
from run_diagnostics import instrument
MyAgent=instrument(Controller)
from vis_nav_sdk import run

if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = cli.parser(__doc__)
    parser.add_argument("--data", help="exploration data directory")
    parser.add_argument("--headless", action="store_true", help="run without the viewer")
    parser.add_argument("--motion-seed", type=int, help="local motion-noise replay seed")
    parser.add_argument("--max-steps", type=int, help="local simulator tick budget")
    args = parser.parse_args()
    if args.local is None and (args.motion_seed is not None or args.max_steps is not None):
        parser.error("--motion-seed and --max-steps are local-only options")
    challenge = cli.challenge(args)
    agent = MyAgent(
        cli.exploration_data(args, args.data),
        cache_dir=Path("cache") / challenge.replace(":", "-"),
    )
    options = cli.run_options(args)
    options["viewer"] = not args.headless
    if args.motion_seed is not None:
        options["motion_seed"] = args.motion_seed
    if args.max_steps is not None:
        options["max_steps"] = args.max_steps
    run(agent, challenge, **options)




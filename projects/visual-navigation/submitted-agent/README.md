# Verified 7/10 midterm agent

## Entry point for the TA

Run source/my_agent.py from this agent_code folder. It automatically imports
the supporting Python modules; do not run them separately. Keep source/ and
tools/ together when zipping this folder. After configuring the external model
resources in MODEL_SETUP.md and your own SDK .env, the standard command is:

    uv run python source/my_agent.py

If the external resources live elsewhere, use the model-location arguments
shown under Runtime requirements below. No policy code has been renamed or changed.

This package contains the preserved agent associated with the recorded 7/10
midterm run: 1,418 ticks, SPL 0.6191819464, 0.035700884 m, 0.094456513 degrees.
It contains no A* agent. All 37 original Python policy/support modules match
the frozen snapshot byte for byte. source/my_agent.py is an added SDK launcher
that selects external resource locations; it does not change the policy.

## Submission

Zip this entire agent_code folder, including source/ and tools/. Upload that ZIP
as the code field. Upload the report slides as a separate PDF. Do not add .env,
data, images, caches or weights. The optional reconstruction discussed in the
slides is not included in this code package and is not the source of this run.

## Runtime requirements

Use the course SDK kit environment, or run uv sync (add --group local for local
simulation). The SDK comes from the configured official course package index.
See MODEL_SETUP.md: the custom PPO checkpoint and official LightGlue weights
remain external because the submission instructions exclude weights. Missing
model files prevent execution; this archive is not self-contained without them.

Standard SDK invocation, after supplying the resources and your own .env:

    uv run python source/my_agent.py --ppo-checkpoint PATH_TO_PPO.pt --matcher-cache PATH_TO_HF_CACHE

Local only, no server attempt:

    uv run --group local python source/my_agent.py --local 7 --data PATH_TO_EXISTING_LOCAL_7_DATA --ppo-checkpoint PATH_TO_PPO.pt --matcher-cache PATH_TO_HF_CACHE

Use --cache-dir PATH to select an existing matching public-image cache. Omitting
it builds the ordinary image graph and odometry under cache/<challenge>. The
recorded 7/10 route additionally used image_verified_links.json generated from
public exploration RGB. Cache contents are excluded from the code submission.
An empty or differently rebuilt cache can yield a different route and score.
The recorded score is evidence for the preserved implementation and its recorded
inputs/cache, not a guarantee for an uncached machine or a new maze.

## Rebuild image-derived connections without a server session

Use downloaded exploration images and a JSON file containing camera_K obtained
from the SDK's public camera metadata. Each command below uses only those public
inputs. All outputs belong in private runtime directories, outside this upload.

    python tools/prepare_public_images.py --data DATA --cache CACHE --camera-json CAMERA_JSON
    python tools/discover_image_connections.py --data DATA --cache CACHE --out WORK --camera-json CAMERA_JSON --stride 8
    python tools/verify_image_connections.py --data DATA --cache CACHE --links WORK/diverse_verified_links.json --out WORK/strict_links.json --camera-json CAMERA_JSON
    python tools/export_image_connections.py --data DATA --verified WORK/strict_links.json --out CACHE/image_verified_links.json --camera-json CAMERA_JSON

The discovery stride is an offline search-budget choice, not a goal location.
All accepted links require image geometry and consistency support. Rebuilding
with different versions, candidate budgets or numeric seeds need not reproduce
the exact accepted link set of the recorded run. No evaluator pose is an input.

## Diagnostics

The included run_diagnostics.py logs SDK RGB, camera calibration, targets, agent
decisions and inferred localization. It does not read true simulator poses.
Logs and images are generated at runtime and are not included in this upload.
The local evaluation harness and evaluator pose traces are excluded entirely.

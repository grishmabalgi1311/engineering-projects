# Underwater Soft Robotics â€” Sim-to-Real Modeling

A CAD-derived MuJoCo model of an 18-segment TPU 95A tentacle for investigating compliance and underwater dynamics.

**Tools:** Python / MuJoCo / TPU 95A / CAD-derived geometry  
**Status:** Ongoing research

## Contribution and documented behavior

- Built the segmented model with 17 passive hinge joints and two tendon elements.
- Parameterized effective joint stiffness and damping to simulate deflection, spring-back and settling.
- Implemented buoyancy and hydrodynamic-drag force variants; developing a sim-to-real calibration workflow.

## Files

- [media/simulator-preview.png](media/simulator-preview.png)
- [source/README.txt](source/README.txt)
- [source/build_model.py](source/build_model.py)
- [source/config.json](source/config.json)
- [source/geometry/meshes/seg01.obj](source/geometry/meshes/seg01.obj)
- [source/geometry/meshes/seg02.obj](source/geometry/meshes/seg02.obj)
- [source/geometry/meshes/seg03.obj](source/geometry/meshes/seg03.obj)
- [source/geometry/meshes/seg04.obj](source/geometry/meshes/seg04.obj)
- [source/geometry/meshes/seg05.obj](source/geometry/meshes/seg05.obj)
- [source/geometry/meshes/seg06.obj](source/geometry/meshes/seg06.obj)
- [source/geometry/meshes/seg07.obj](source/geometry/meshes/seg07.obj)
- [source/geometry/meshes/seg08.obj](source/geometry/meshes/seg08.obj)
- [source/geometry/meshes/seg09.obj](source/geometry/meshes/seg09.obj)
- [source/geometry/meshes/seg10.obj](source/geometry/meshes/seg10.obj)
- [source/geometry/meshes/seg11.obj](source/geometry/meshes/seg11.obj)
- [source/geometry/meshes/seg12.obj](source/geometry/meshes/seg12.obj)
- [source/geometry/meshes/seg13.obj](source/geometry/meshes/seg13.obj)
- [source/geometry/meshes/seg14.obj](source/geometry/meshes/seg14.obj)
- [source/geometry/meshes/seg15.obj](source/geometry/meshes/seg15.obj)
- [source/geometry/meshes/seg16.obj](source/geometry/meshes/seg16.obj)
- [source/geometry/meshes/seg17.obj](source/geometry/meshes/seg17.obj)
- [source/geometry/meshes/seg18.obj](source/geometry/meshes/seg18.obj)
- [source/model/arm_reference.xml](source/model/arm_reference.xml)
- [source/model/two_tentacles_ball.xml](source/model/two_tentacles_ball.xml)
- [source/run_project2.py](source/run_project2.py)
- [source/simulator.py](source/simulator.py)
- [source/water.py](source/water.py)

## Setup and verification boundary

The `source/` folder preserves an isolated MuJoCo pixel-water variant with its original model and OBJ meshes. Review source/README.txt and config.json. Requires Python, MuJoCo, NumPy, and its declared viewer dependencies. The preview is a saved simulator image, not new physical validation. Effective stiffness, damping, buoyancy, and drag require experimental calibration. No trained RL policy is claimed.

## Scope and attribution

Model construction and force implementations are established; physical calibration and sim-to-real validation remain ongoing.

Team/course projects include inherited frameworks and collaborators. File inclusion is not a claim of sole authorship. Source files retain their original comments and attribution. No blanket license is applied to third-party or jointly authored material.

## Next step

Compare simulated trajectories with physical experiments and identify model discrepancies.

[Portfolio case study](https://grishmabalgi1311.github.io/grishma-portfolio/#project=soft-tentacle-simulation) · [Grishma Balgi](https://github.com/grishmabalgi1311)

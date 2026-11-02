# UR10e kinematics in MuJoCo

Position and velocity kinematics explored on a UR10e model in a supplied MuJoCo simulation framework.

**Tools:** Python / NumPy / MuJoCo / Numerical inverse kinematics  
**Status:** Coursework implementation

## Contribution and documented behavior

- Implemented DH-based forward-position calculations and numerical inverse-position solutions.
- Explored Jacobian-based velocity relationships and pseudoinverse joint-velocity calculations.
- Visualised robot motion using a supplied coursework simulation and helper functions.

## Files

- [media/ur10e_FK_position.gif](media/ur10e_FK_position.gif)
- [media/ur10e_FK_velocity.gif](media/ur10e_FK_velocity.gif)
- [media/ur10e_IK_position.gif](media/ur10e_IK_position.gif)
- [media/ur10e_IK_velocity.gif](media/ur10e_IK_velocity.gif)
- [notebooks/FORFinalProject.ipynb](notebooks/FORFinalProject.ipynb)

## Setup and verification boundary

Open the notebook in a Python environment with NumPy, MuJoCo and its imported coursework utilities. The supplied robot framework and MuJoCo Menagerie model are third-party dependencies and are not republished here. Notebook outputs are cleared; saved GIFs are original visualization artifacts. This curation did not re-execute the complete notebook.

## Scope and attribution

Coursework implementation within a provided framework; the robot model and starter utilities are not claimed as original work.

Team/course projects include inherited frameworks and collaborators. File inclusion is not a claim of sole authorship. Source files retain their original comments and attribution. No blanket license is applied to third-party or jointly authored material.

## Next step

Extend evaluation to multiple poses, solver convergence and near-singular configurations.

[Portfolio case study](https://grishmabalgi1311.github.io/grishma-portfolio/#project=ur10e-kinematics) · [Grishma Balgi](https://github.com/grishmabalgi1311)

# Obstacle detection & automated steering

A Pythonâ€“MATLAB/Simulink simulation connecting synthetic obstacle proximity, collision classification and lane-change logic.

**Tools:** Python / MATLAB / Simulink / Decision trees / Simulation  
**Status:** Undergraduate team project

## Contribution and documented behavior

- Modelled a three-lane driving environment with virtual obstacle-distance inputs and vehicle motion.
- Used a decision-tree classification workflow on simulation-generated data.
- Integrated Python model predictions with MATLAB-based steering and lane-change visualisation.

## Files

- [reports/Obstacle_Detection_and_Automated_Steering.pdf](reports/Obstacle_Detection_and_Automated_Steering.pdf)
- [source/Obstacle_avoidance_bot.slx](source/Obstacle_avoidance_bot.slx)
- [source/data_collection.m](source/data_collection.m)
- [source/matlabtopython.m](source/matlabtopython.m)
- [source/obstaclepythonpyfile.py](source/obstaclepythonpyfile.py)

## Setup and verification boundary

MATLAB/Simulink files and Python scripts are archived together. The Python script imports pandas, NumPy, joblib, scikit-learn, and matplotlib and expects obstacle_data.csv. Training data, serialized models and proprietary toolboxes are not vendored. This is a simulated three-lane scenario; no physical LiDAR or road deployment is claimed.

## Scope and attribution

Simulation-only project: no physical LiDAR integration or on-vehicle autonomous driving is claimed. Reported classification accuracy is not presented as real-world safety performance.

Team/course projects include inherited frameworks and collaborators. File inclusion is not a claim of sole authorship. Source files retain their original comments and attribution. No blanket license is applied to third-party or jointly authored material.

## Next step

Evaluate on independent simulation scenarios and compare against a simple distance-threshold baseline.

[Portfolio case study](https://grishmabalgi1311.github.io/grishma-portfolio/#project=automated-steering) · [Grishma Balgi](https://github.com/grishmabalgi1311)

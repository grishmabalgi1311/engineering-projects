# Gesture tracking & two-finger prototype

MediaPipe gesture tracking and a two-finger actuation prototype, developed and tested as separate subsystems.

**Tools:** Python / MediaPipe / Motor actuation / Mechanical prototyping  
**Status:** Prototype Â· Integration unfinished

## Contribution and documented behavior

- Implemented a MediaPipe-based hand-gesture tracking pipeline.
- Developed a two-finger mechanical actuation prototype.
- Tested vision and hardware separately; gesture-to-actuator integration was not attempted before the project ended.

## Files

- [firmware/mechatronicsprojectide/mechatronicsprojectide.ino](firmware/mechatronicsprojectide/mechatronicsprojectide.ino)
- [matlab/closedloop.mlx](matlab/closedloop.mlx)
- [matlab/dc_motor_pd_encoder.slx](matlab/dc_motor_pd_encoder.slx)
- [matlab/openloop.mlx](matlab/openloop.mlx)
- [reports/E2_Report_Group01_Single-DOFGestureControlledRoboticFinger.pdf](reports/E2_Report_Group01_Single-DOFGestureControlledRoboticFinger.pdf)
- [vision/mediapipecode.py](vision/mediapipecode.py)

## Setup and verification boundary

Vision code and Arduino firmware were tested as separate subsystems in the original project. MATLAB/Simulink models are included as development artifacts. The final scope was a two-finger prototype, not an integrated gesture-controlled hand. Do not infer end-to-end control or closed-loop performance from the presence of both code folders.

## Scope and attribution

The linked course report describes a single-DOF stage and modelling work. The final user-confirmed scope was two fingers with no completed end-to-end integration; full-hand and settling-time claims are excluded.

Team/course projects include inherited frameworks and collaborators. File inclusion is not a claim of sole authorship. Source files retain their original comments and attribution. No blanket license is applied to third-party or jointly authored material.

## Next step

Future integration could map tracked gestures to motor commands and evaluate closed-loop behaviour where feedback is available.

[Portfolio case study](https://grishmabalgi1311.github.io/grishma-portfolio/#project=gesture-fingers) · [Grishma Balgi](https://github.com/grishmabalgi1311)

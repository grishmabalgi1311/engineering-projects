# Robotic-fin co-design

A physical platform for fabricating fins, varying actuation and comparing force measurements.

**Tools:** Python / USB / Silhouette Portrait 4 / Arduino / HX711 / Dynamixel  
**Status:** Ongoing research

## Contribution and documented behavior

- Fabricated the test setup and automated coordinate-based cutting through Python USB control.
- Developed the motor-control interface, load-cell firmware, recording and analysis workflow.
- Added selectable waveform, amplitude and frequency, with commanded/actual motor-position logging.

## Files

- [firmware/ForceServo/ForceServo.ino](firmware/ForceServo/ForceServo.ino)
- [src/controller.py](src/controller.py)
- [src/dynamixel_address_book.py](src/dynamixel_address_book.py)
- [src/dynamixel_controller.py](src/dynamixel_controller.py)
- [src/fish_fin_sway.py](src/fish_fin_sway.py)
- [src/plot_all_trials.py](src/plot_all_trials.py)
- [src/plot_trial.py](src/plot_trial.py)

## Setup and verification boundary

The Python GUI needs Python, tkinter, pyserial, pandas, matplotlib, and dynamixel-sdk. Arduino firmware needs its declared libraries. Review COM ports, servo configuration, power, and geometry before any physical run. This archive was not connected to the rig. Logging and cutting/handling are different subsystems; an unattended full test cycle is not claimed.

## Scope and attribution

Measured force and a provisional ranking score support exploration. Calibrated net thrust and electrical efficiency have not been established.

Team/course projects include inherited frameworks and collaborators. File inclusion is not a claim of sole authorship. Source files retain their original comments and attribution. No blanket license is applied to third-party or jointly authored material.

## Next step

Refine force calibration and repeatability; complete UR5e fin handling and choose the optimisation method.

[Portfolio case study](https://grishmabalgi1311.github.io/grishma-portfolio/#project=robotic-fin-codesign) · [Grishma Balgi](https://github.com/grishmabalgi1311)

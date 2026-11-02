# Circuit-breaker fault detection

Vibration and acoustic sensing for investigating circuit-breaker spring-removal conditions.

**Tools:** Python / Arduino / Sensor acquisition / Classical ML  
**Status:** Undergraduate team project

## Contribution and documented behavior

- Developed sensor acquisition, Python preprocessing and classification workflows for four spring-removal conditions.
- Compared five classifiers using an 80:20 train-test split on synthetic sensor data.
- Random Forest and XGBoost achieved 99.98% synthetic test accuracy, with two errors in 12,000 test samples.

## Files

- [simulation/ACB_RealTime_Simulation.slx](simulation/ACB_RealTime_Simulation.slx)
- [simulation/simulated_ACB_fault_data.m](simulation/simulated_ACB_fault_data.m)

## Setup and verification boundary

The MATLAB/Simulink artifacts document the simulation portion of a team fault-detection study. The final-year report recorded 99.98% accuracy on synthetic test data (2 errors / 12,000 samples). No claim of industrial generalization, on-device inference, or independent physical-test accuracy is made. Confidential internship documents and industrial data are not included.

## Scope and attribution

The 99.98% result is from synthetic-data evaluation in the final-year report; it does not establish industrial deployment accuracy.

Team/course projects include inherited frameworks and collaborators. File inclusion is not a claim of sole authorship. Source files retain their original comments and attribution. No blanket license is applied to third-party or jointly authored material.

## Next step

Evaluate on independently collected physical trials and quantify sensitivity to operating conditions.

[Portfolio case study](https://grishmabalgi1311.github.io/grishma-portfolio/#project=circuit-breaker-fault-detection) · [Grishma Balgi](https://github.com/grishmabalgi1311)

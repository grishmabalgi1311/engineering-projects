# KinetiKey

An STM32 gesture-combination lock using IMU data, filtering and memory-conscious sequence matching.

**Tools:** C++ / STM32L475 / Mbed OS / I2C / Dynamic time warping  
**Status:** Course project

## Contribution and documented behavior

- Implemented IMU acquisition over 400 kHz I2C, low-pass filtering and matching of an enrolled three-gesture sequence.
- Bounded dynamic time warping with 200-sample gesture buffers, a 30-sample search band and a two-row workspace.
- Implemented interrupt-driven enrollment, LED feedback and three-attempt lockout.

## Files

- [platformio.ini](platformio.ini)
- [src/main.cpp](src/main.cpp)

## Setup and verification boundary

Target: STM32L475 IoT Discovery (`disco_l475vg_iot01a`), PlatformIO + Mbed OS. The supplied platformio.ini is the original target configuration. Build with `pio run` after installing the toolchain. This curation did not compile, flash, or test hardware. Enrollment and recognition error rates have not been benchmarked.

## Scope and attribution

Source implementation is available locally; formal recognition and timing benchmarks are still to be documented.

Team/course projects include inherited frameworks and collaborators. File inclusion is not a claim of sole authorship. Source files retain their original comments and attribution. No blanket license is applied to third-party or jointly authored material.

## Next step

Measure recognition errors, processing time, sampling jitter and sensitivity to sensor orientation.

[Portfolio case study](https://grishmabalgi1311.github.io/grishma-portfolio/#project=kinetikey) · [Grishma Balgi](https://github.com/grishmabalgi1311)

# ECG acquisition & heart rate

Periodic ECG sampling, circular buffering and an ARM assembly peak-counting routine.

**Tools:** C++ / ARM Thumb-2 / STM32 / Mbed Ticker / AD8232  
**Status:** Course project

## Contribution and documented behavior

- Implemented periodic acquisition with a 1,080-sample circular buffer and lead-off inputs.
- Developed an assembly routine for threshold-based peak counting with a refractory interval.
- Maintained a C reference implementation for the heart-rate calculation.

## Files

- [platformio.ini](platformio.ini)
- [src/getHR.S](src/getHR.S)
- [src/main.cpp](src/main.cpp)

## Setup and verification boundary

Target: STM32 with Mbed OS; AD8232 input on PA_0 and lead-off pins PB_0/PB_1. The original PlatformIO configuration, C++ acquisition code, and Thumb-2 assembly are included. The configuration is source evidence, not a new hardware verification. Educational prototype; not validated for clinical use.

## Scope and attribution

An educational embedded prototype; clinical accuracy or medical-device validation is not claimed.

Team/course projects include inherited frameworks and collaborators. File inclusion is not a claim of sole authorship. Source files retain their original comments and attribution. No blanket license is applied to third-party or jointly authored material.

## Next step

Document reference-signal comparisons and buffer/timing behaviour.

[Portfolio case study](https://grishmabalgi1311.github.io/grishma-portfolio/#project=ecg-monitor) · [Grishma Balgi](https://github.com/grishmabalgi1311)

# FlexiClimber 2.0

A pivot extension to an existing beam-traversal platform, with embedded sensing and actuator sequencing.

**Tools:** SolidWorks / C / Parallax Propeller / Arduino / Bluetooth  
**Status:** Team course project

## Contribution and documented behavior

- Worked on a pivot mechanism extending an inherited rack-and-pinion beam robot.
- Developed mechanical and embedded components for reorientation and remote operation.
- Worked with concurrent sensing, command parsing and actuator-control routines.

## Files

- [firmware/arduino_pivot.ino](firmware/arduino_pivot.ino)
- [firmware/arduino_pivot_bitwise.ino](firmware/arduino_pivot_bitwise.ino)
- [firmware/flexibot_2.1.c](firmware/flexibot_2.1.c)

## Setup and verification boundary

Propeller C source uses SimpleIDE/simpletools, servo, and fdserial libraries. Arduino pivot routines are archived separately. Pin mappings and actuator timings must be checked against the actual robot. The inherited traversal platform and team contributions are acknowledged; full pivot integration remained unfinished. These historical sources were not flashed or physically retested.

## Scope and attribution

The original platform is prior student work. Full pivot integration remained unfinished.

Team/course projects include inherited frameworks and collaborators. File inclusion is not a claim of sole authorship. Source files retain their original comments and attribution. No blanket license is applied to third-party or jointly authored material.

## Next step

Document repeatable traversal tests and final integrated pivot behaviour.

[Portfolio case study](https://grishmabalgi1311.github.io/grishma-portfolio/#project=flexiclimber) · [Grishma Balgi](https://github.com/grishmabalgi1311)

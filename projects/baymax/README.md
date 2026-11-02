# Baymax desk companion

A dual-board companion robot prototype combining camera and voice interfaces with display, light and motor outputs.

**Tools:** Python / Raspberry Pi 3B / Jetson Nano / Ethernet / Gemini API / L298N  
**Status:** Team course project

## Contribution and documented behavior

- Integrated a Raspberry Pi 3B and Jetson Nano over Ethernet with camera, audio, display and motor interfaces.
- Worked with L298N-driven DC motors, RGB LED control and separate 12 V / 5 V power distribution.
- Explored local language-model and open-source speech components, then used a cloud Gemini API for conversational responses.

## Files

- [display/animated_eyes.py](display/animated_eyes.py)

## Setup and verification boundary

The included pygame display component is one original prototype component, not the complete Raspberry Pi/Jetson application. The team prototype used a Gemini cloud API for conversational responses. Service credentials, recordings, and personal data are excluded. Run the display component in a development window with Python + pygame; final speech/model versions and response latency are not established.

## Scope and attribution

Conversational responses used a cloud API. Exact model version and final speech implementation are not documented here; learning improvements or user-study outcomes have not been measured.

Team/course projects include inherited frameworks and collaborators. File inclusion is not a claim of sole authorship. Source files retain their original comments and attribution. No blanket license is applied to third-party or jointly authored material.

## Next step

Document the final board-level workload, message protocol and measured response latency.

[Portfolio case study](https://grishmabalgi1311.github.io/grishma-portfolio/#project=baymax) · [Grishma Balgi](https://github.com/grishmabalgi1311)

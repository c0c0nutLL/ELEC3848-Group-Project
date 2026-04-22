# Final Integrated Firmware

This directory is reserved for the final Arduino firmware that integrates the full proposal workflow into one main sketch.

Intended scope of the final integrated firmware:

- base locomotion
- gyro-assisted straight travel
- front-ultrasonic docking and close-range safety
- station verification
- arm manipulation
- cargo handling / unlock interaction
- telemetry upload to Jetson Nano
- status feedback for the dashboard

## Purpose Of This Directory

The current `proposal_amr` area already contains several useful sketches, but they are still split by role:

- telemetry / manual bridge demo
- interrupt / resume demo
- motor calibration assistant
- obstacle-related experiments

This new directory is the reserved location for the final consolidated Arduino implementation.

The goal is to avoid mixing:

- test sketches
- calibration tools
- historical demos
- the final demo / submission firmware

## Expected Final Contents

Recommended structure for this directory:

- one main final sketch
- any small local helper files that belong only to the final integrated firmware
- a short wiring / dependency note if the final build needs one

Suggested main sketch name:

- `final_integrated_firmware.ino`

If the team later chooses a different final name, update this README and `plan.md` together.

## Source Inputs For The Final Sketch

The final firmware here is expected to reuse proven work from:

- `firmware/arduino/required_function_c1/`
- `firmware/arduino/proposal_amr/telemetry_serial_demo/`
- `firmware/arduino/proposal_amr/interrupt_resume_demo/`
- `firmware/arduino/proposal_amr/motor_calibration_assistant/`
- `firmware/arduino/HW-868/`

Use those directories as references and test assets, but do not treat them as the final integrated submission location.

## Directory Rule

Once the team starts building the final all-in-one Arduino version, this directory should become the single obvious place to look first.

That means:

- new final-mission logic should go here
- temporary experiments should stay in their own test folders
- calibration tools should stay outside this directory

## Current Status

This directory is currently a placeholder and documentation anchor only.

The final integrated Arduino code has not been moved here yet.

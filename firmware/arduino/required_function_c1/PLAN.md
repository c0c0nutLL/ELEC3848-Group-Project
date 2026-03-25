# Required Function C1 Plan

## Summary

Build a pass-first, fully automatic parking flow on Arduino Mega that starts from the marked 1 m position, drives into the parking zone, stops at 8 cm from the wall without touching it, corrects heading to become parallel to the wall, and continuously reports distance and angle to both OLED and the computer.

Primary strategy:

- do not depend on the color sensor for the first passing version
- use ultrasonic, gyroscope, motor control, OLED, and serial telemetry as the minimum viable stack
- keep Bluetooth only as a debug or trigger fallback, not part of the main scoring path

Target implementation shape:

- one main sketch for the demo flow
- independent sensor and motor test sketches remain for calibration and debugging
- a simple state machine controls the full-auto behavior

## Implementation Changes

### 1. Main control program

Create a single main sketch for Required Function, named `required_function_c1.ino`, built around these states:

- `IDLE`
- `START_DELAY`
- `APPROACH_FAST`
- `APPROACH_SLOW`
- `STOP_AT_TARGET`
- `PARALLEL_CORRECT`
- `HOLD_AND_REPORT`

State behavior:

- `IDLE`: initialize OLED, gyro, ultrasonic, and telemetry
- `START_DELAY`: use a short fixed delay after power-on or start trigger so the car is stable before moving
- `APPROACH_FAST`: drive forward from the 1 m start position using ultrasonic distance feedback
- `APPROACH_SLOW`: slow down near the wall to reduce overshoot
- `STOP_AT_TARGET`: stop when distance reaches the target tolerance band
- `PARALLEL_CORRECT`: use gyro-based heading correction to minimize angular error and align parallel to the wall
- `HOLD_AND_REPORT`: keep motors stopped and continue showing final distance and heading on OLED and serial

Decision-complete behavior:

- full-auto means one start action only; after start, no manual steering
- use only forward motion for the first pass version unless overshoot recovery is needed during testing
- if the car overshoots badly in practice, allow one limited reverse recovery branch only if testing proves it is necessary
- do not include a color-sensor-dependent path in v1

### 2. Motion and parking logic

Use the existing motor direction mapping from `reference.ino`.

Control rules:

- use two-speed forward approach
- fast approach when distance is comfortably above target
- slow approach when near the wall

Initial working thresholds:

- target distance: 8 cm
- slow-down threshold: around 18 to 25 cm
- acceptable stop tolerance: around +/- 1 cm

Stop condition:

- stop immediately once measured distance enters the target band
- do not continue nudging toward the wall once inside the band

Safety condition:

- if ultrasonic read fails or becomes unstable, stop the car and report sensor fault
- if distance becomes dangerously small, hard stop

Pass-first tuning rule:

- prefer slightly conservative stopping over aggressive final creep
- tune for no wall contact before tuning for tighter distance

### 3. Parallel correction logic

Use `MPU6050` only for short-duration heading correction.

Behavior:

- calibrate gyro bias at boot while the car is stationary
- zero heading before movement starts
- after distance stop, compare current heading to the stored forward heading reference
- rotate left or right in place with low PWM until heading error is inside tolerance
- stop and hold once heading is inside tolerance band

Initial working tolerances:

- heading tolerance: around 1.5 to 3 degrees
- use low rotation PWM for correction to avoid oscillation

Important implementation rule:

- the parallel target is the heading established when the car starts straight
- do not attempt long-term gyro navigation; only use gyro for short correction at the end

### 4. Display and telemetry

Required by the PDF: show sensor data on both OLED and computer.

OLED:

- continuously show current state
- show ultrasonic distance in cm
- show heading angle in degrees
- in final state, show final parked distance and final heading error
- keep layout simple and readable from outside

Computer telemetry:

- output plain serial text at fixed intervals
- include at minimum:
  - timestamp or loop count
  - current state
  - distance
  - heading
  - stop, correction, and fault messages
- keep the output easy to copy into the control-description report

### 5. Calibration and support files

Keep and use the current test sketches as dedicated calibration tools:

- `basic_motor_control.ino`: verify actual wheel directions and PWM behavior
- `motion_ultrasonic_parking_test.ino`: verify real sensor pins, distance quality, and stop thresholds
- `gyro_parallel_correction_test.ino`: verify gyro initialization, bias, and turning direction
- `color_sensor_red_green_black_test.ino`: keep as optional future work, not in the first-pass main flow

Use the pin-mapping workbook as source of truth before merging everything into the main sketch.

## Interfaces / Inputs / Outputs

### Main inputs

- front-facing `HC-SR04` ultrasonic distance
- `MPU6050` heading change
- optional start signal:
  - auto-start after boot delay is recommended for the simplest demo
  - one-button or one-command start is acceptable only if the demo format allows it

### Main outputs

- motor PWM and direction signals
- OLED distance and angle display
- serial telemetry to the computer

### Public file intent

Recommended final file set for this phase:

- `reference.ino`: untouched original reference
- test sketches: kept separate for calibration
- `required_function_c1.ino`: integrated demo program
- `PLAN.md`: execution checklist and tuning notes

## Test Plan

### Bench tests

- verify each motor direction matches expected motion
- verify front ultrasonic distance is monotonic and stable from 100 cm down to 8 cm
- verify gyro can zero while stationary and report heading drift within acceptable short-run range
- verify OLED updates correctly and is readable
- verify serial output logs distance and angle continuously

### Integrated floor tests

- start at 1 m from wall and confirm the car reaches the stop band without collision
- repeat at least 5 to 10 times to observe consistency
- check that the car stops near 8 cm and does not touch the wall during approach
- check that final heading correction reduces visible skew
- check that telemetry and OLED still update during and after parking

### Acceptance criteria

- full-auto from start to parked state
- no wall contact
- final distance near 8 cm
- final orientation visibly parallel to wall
- distance and angle visible on OLED and serial output
- behavior is repeatable enough for demo confidence

## Assumptions and Defaults

- pass-first means prioritize reliability over extra features
- full-auto means no manual steering once the routine starts
- the first scored version will not rely on color sensor or Jetson Nano
- Bluetooth is optional debug support only
- the heading reference for parallel is the initial straight heading at start
- the pin mapping in the workbook will be finalized before integrating the main sketch
- if start mode is not explicitly constrained by the instructor, default to auto-start after a short boot delay because it is the simplest full-auto interpretation

## Implementation Order

1. Finalize the pin mapping sheet.
2. Validate motor directions.
3. Validate ultrasonic stop thresholds.
4. Validate gyro bias and correction direction.
5. Build the integrated state machine sketch.
6. Add OLED and serial telemetry.
7. Tune stop distance and correction tolerance.
8. Repeat full demo runs and record stable settings.

## Deliverables Checklist

- working `required_function_c1.ino`
- one-page control description
- code submission package
- stable demo-ready parameter values

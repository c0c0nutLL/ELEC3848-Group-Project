# Arduino Fixed-Strafe Obstacle Avoidance Test

This directory contains the Arduino-side code for the independent fixed-strafe obstacle avoidance test.

Main sketch:

- `firmware/arduino/proposal_amr/obstacle_avoidance_strafe_test/obstacle_avoidance_strafe_test.ino`

## What This Sketch Does

This sketch is the real-time controller for:

- front ultrasonic blocking confirmation
- gyro heading hold
- wait-first obstacle handling
- plan A only: fixed strafe + forward + return + heading recovery

It is separate from the main mission firmware.

## Current State Machine

- `STOPPED`
- `CLEAR`
- `WAIT_OBSTACLE`
- `DECIDE_SIDE`
- `STRAFE_LEFT`
- `STRAFE_RIGHT`
- `PASS_FORWARD`
- `RETURN_TO_PATH`
- `RECOVER_HEADING`
- `DONE`
- `FAILSAFE`

## Why This Is Only A Conservative Test

The side ultrasonic sensors are removed.
So this sketch does not know whether the left or right side is really free.

That means the current strafe action is only:

- a template maneuver
- for controlled test scenes
- not a complete autonomous avoidance system

It can only do:

- close-range front blocking detection
- heading stabilization
- fixed-time lateral motion
- fixed-time return motion

It cannot do:

- real-time side clearance verification
- exact path-center recovery
- map-based or position-based avoidance

## Hardware Reuse

This sketch reuses the current hardware references:

- front ultrasonic pair
  - right front: `TRIG=6`, `ECHO=4`
  - left front: `TRIG=10`, `ECHO=7`
- start button: `33`
- MPU6050 gyro
- OLED
- Arduino Mega motor pins from the Required Function baseline

## Serial Protocol

Arduino accepts:

```text
START
STOP
RESET
OBS <TYPE> <ZONE> <CONFIDENCE>
```

Examples:

```text
OBS PERSON LEFT 82
OBS OBJECT CENTER 61
OBS CLEAR NONE 0
```

Arduino sends JSON telemetry lines back over the same serial link.

## OLED Output

OLED shows:

- current state
- front distance
- current obstacle type
- current action
- current heading error

## How To Run

1. Upload `obstacle_avoidance_strafe_test.ino` to Arduino Mega.
2. Open serial at `115200` if you want to inspect telemetry manually.
3. Start the test in one of two ways:
   - send `START`
   - or press the current start button
4. To stop:
   - send `STOP`
5. To clear failsafe and return to stopped state:
   - send `RESET`
   - or press the start button once while in failsafe

## Important Tuning Parameters

These parameters are intentionally exposed as constants in the sketch:

- person wait time
- object wait time
- strafe duration
- pass duration limits
- return duration
- heading recovery timeout
- forward / pass / strafe PWM

They must be tuned on the real chassis.

## Biggest Remaining Risk

The biggest remaining risk is not software logic.
It is the missing side-clearance sensing.

Even with correct code, the robot can still choose a blocked side because it has no sensor to verify that space.

# Obstacle Avoidance Arduino Test

## Deprecation Notice

This test path is now kept only as a historical prototype.

Reason:

- it depends on the removed side ultrasonic sensors
- the current hardware baseline keeps only the front ultrasonic pair

Do not treat this directory as a current upload target.

This directory contains the Arduino-side code for the independent:

- YOLO + ultrasonic obstacle detection
- wait-first behavior
- local active avoidance

test path.

It is not the main mission sketch.

Main sketch:

- `firmware/arduino/proposal_amr/obstacle_avoidance_test/obstacle_avoidance_test.ino`

## Historical Hardware Reuse

This sketch reuses the current project hardware references:

- front dual ultrasonic
  - right front: `TRIG=6`, `ECHO=4`
  - left front: `TRIG=10`, `ECHO=7`
- side ultrasonic
  - left side: `TRIG=28`, `ECHO=25`
  - right side: `TRIG=30`, `ECHO=32`
  - removed from the current hardware on 2026-04-21
- MPU6050 gyro
- OLED
- Arduino Mega motor pins from the Required Function baseline

## Test Strategy

This sketch does not try to perform full autonomous navigation.
It only validates a conservative local bypass chain:

1. move forward
2. detect near blocking with front ultrasonic
3. stop and wait
4. if still blocked, choose a safer side
5. sidestep
6. pass forward
7. sidestep back
8. recover heading

## Why This Is Only A Test Version

What is reliable enough here:

- local near-obstacle stop decision
- wait-before-avoid behavior
- simple side choice using left/right distance
- heading recovery using gyro
- serial telemetry for Jetson-side observation

What is still only a conservative prototype:

- bypassing a person in arbitrary poses
- avoiding obstacles outside a simple local corridor-like scene
- returning exactly to a global path after avoidance

The sidestep distance is not based on map or odometry.
It is based on:

- current side-wall distance
- target local shift
- timeout protection

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
OBS PERSON CENTER 82
OBS OBJECT LEFT 67
OBS CLEAR NONE 0
```

Arduino sends JSON telemetry lines back over the same serial link.

## OLED Output

OLED shows:

- current state
- front distance
- left/right side distance
- obstacle type
- current action
- avoid side

## If You Revisit This Prototype

Before reusing this idea on current hardware, the sidestep logic must be
redesigned so it does not depend on side ultrasonic distance sensing.

## Historical Run Notes

1. Upload `obstacle_avoidance_test.ino` to Arduino Mega.
2. Open serial at `115200` if you want to inspect telemetry manually.
3. Start the test in one of two ways:
   - send `START`
   - or press the current start button
4. To stop:
   - send `STOP`
5. To clear failsafe and return to stopped state:
   - send `RESET`
   - or press the start button once while in failsafe

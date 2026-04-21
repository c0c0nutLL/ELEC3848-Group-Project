# YOLO + Ultrasonic Obstacle Avoidance Test

## Deprecation Notice

This Jetson-side prototype is now a historical companion document only.

Reason:

- its Arduino counterpart depends on the removed side ultrasonic sensors
- the current robot hardware no longer supports that local sidestep strategy as written

Keep it only as an experiment record, not as a current integration target.

This directory contains the Jetson-side code for an independent obstacle avoidance prototype.

It is intentionally separate from:

- the main mission program
- `software/jetson_dashboard`
- the proposal integrated state machine

Main script:

- `software/jetson_yolo_tests/obstacle_avoidance_test/obstacle_avoidance_yolo_bridge.py`

## What This Test Does

Jetson Nano:

- reads the CSI camera
- runs YOLO
- reduces the result to a simple obstacle message
- sends that message over USB serial to Arduino

Arduino:

- reads front and side ultrasonics plus MPU6050 gyro
- decides whether the robot is really blocked
- stops and waits first
- only then performs a conservative local bypass maneuver

## Important Environment Notes

- Current YOLO runtime depends on Docker on Jetson Nano.
- Host-side `venv` is still not finished.
- Current Docker test with `imshow` is about `8 FPS`.
- `yolo26n.pt` and `yolo11n.pt` currently show similar FPS.
- Future optimization should prioritize:
  - code path and display overhead
  - runtime and pipeline optimization
  - export/runtime changes such as TensorRT
  - not only changing the model name

## Why This Is A Conservative Test Version

This system does not assume:

- map localization
- GPS
- SLAM
- depth camera
- reliable global path planning

What it can realistically do with current hardware:

- detect a likely person in the camera view
- detect a likely generic obstacle in the camera view
- use front ultrasonic distance to confirm near blocking
- wait first instead of immediately steering around
- choose the side with more room using side ultrasonics
- perform a small local bypass using mecanum-style lateral motion
- use gyro to keep heading stable during sidestep, pass, and recovery

What it cannot honestly guarantee yet:

- stable avoidance of every human pose and every obstacle shape
- robust behavior in crowded scenes
- safe path planning outside a simple local corridor-like local scene
- long-range rejoining of a global route after bypass

## Obstacle Definition Used In This Test

This prototype uses the following rules:

- `PERSON`
  - YOLO detects class `person`
  - Arduino only treats it as a real block if front ultrasonic distance is near, for example `<= 50 cm`
- `OBJECT`
  - YOLO detects a non-`person` obstacle candidate
  - Arduino only treats it as a real block if front ultrasonic distance is near
- `UNKNOWN`
  - YOLO is clear or stale, but front ultrasonic still reports a close obstacle
  - Arduino then treats it as a generic unknown near obstacle

This means the final block decision is always made on Arduino using local distance sensing.

## Serial Protocol

Jetson -> Arduino:

```text
START
STOP
RESET
OBS <TYPE> <ZONE> <CONFIDENCE>
```

Examples:

```text
OBS CLEAR NONE 0
OBS PERSON CENTER 82
OBS OBJECT LEFT 67
```

Definitions:

- `TYPE`
  - `CLEAR`
  - `PERSON`
  - `OBJECT`
  - `UNKNOWN`
- `ZONE`
  - `NONE`
  - `LEFT`
  - `CENTER`
  - `RIGHT`
- `CONFIDENCE`
  - integer `0-100`

Arduino -> Jetson:

- JSON telemetry lines for debugging
- state, action, distances, obstacle type, avoid side, heading, and fault fields

## How To Run

Run this script inside the current Jetson YOLO Docker runtime, not the unfinished host `venv`.

### Dependencies inside the runtime

- `ultralytics`
- `opencv-python`
- `pyserial`

If `pyserial` is missing in the container:

```bash
pip3 install pyserial
```

### Example command

```bash
cd software/jetson_yolo_tests/obstacle_avoidance_test
python3 obstacle_avoidance_yolo_bridge.py \
  --model yolo11n.pt \
  --serial /dev/ttyACM0 \
  --baud 115200 \
  --camera 0
```

### Optional display mode

Display is off by default to reduce overhead.
If you want to inspect the annotated image:

```bash
python3 obstacle_avoidance_yolo_bridge.py \
  --model yolo11n.pt \
  --serial /dev/ttyACM0 \
  --camera 0 \
  --show
```

### Optional auto-start

If you want Jetson to send `START` once after serial is ready:

```bash
python3 obstacle_avoidance_yolo_bridge.py \
  --model yolo11n.pt \
  --serial /dev/ttyACM0 \
  --camera 0 \
  --auto-start
```

Notes:

- the current implementation targets the Jetson CSI camera path
- `/dev/video0` is accepted only as a CSI alias for sensor `0`
- there is no separate USB camera branch in this test directory right now

## Current Behavior Summary

Jetson only sends a compact obstacle observation.
Arduino is the real-time controller.

Because current YOLO speed is limited, Jetson is not used as a real-time motion controller.
This split is intentional and correct for the current hardware.

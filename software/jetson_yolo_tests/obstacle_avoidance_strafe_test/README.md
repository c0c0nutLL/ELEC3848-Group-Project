# Fixed-Strafe Obstacle Avoidance Test

This directory contains the Jetson-side code for the independent obstacle avoidance prototype based on:

- YOLO on Jetson Nano
- front ultrasonic confirmation on Arduino
- plan A only: fixed strafe + forward + return + heading recovery

Main script:

- `software/jetson_yolo_tests/obstacle_avoidance_strafe_test/obstacle_avoidance_strafe_bridge.py`

## What This Test System Does

Jetson Nano:

- reads the CSI camera
- runs YOLO
- prioritizes `person`
- also allows a generic non-person obstacle candidate
- reduces the vision result to a simple serial message for Arduino

Arduino Mega:

- reads the front ultrasonic pair
- reads MPU6050 heading
- decides whether the robot is really blocked at close range
- waits first
- if still blocked, performs a fixed strafe template
- tries to recover heading and then return to normal forward driving

## This Test Uses Plan A Only

This implementation uses:

1. fixed strafe to one side
2. forward pass
3. fixed strafe back
4. heading recovery

It does not implement any alternative avoidance strategy.

## Why This Is Only A Conservative Test Version

This is not a full autonomous obstacle avoidance system.

The biggest reason is hardware:

- the side ultrasonic sensors are removed
- there is no map
- there is no encoder-based lateral position estimate
- there is no depth camera

So the robot cannot truly verify whether the chosen side is clear.

That means this test is only valid as a controlled-environment template test.
It should not be described as a complete or reliable avoidance system.

## What The Current Hardware Can Reliably Do

- detect a likely person in the camera view
- detect a likely generic object candidate in the camera view
- use the front ultrasonic pair to confirm near blocking
- stop and wait before moving around the obstacle
- keep heading more stable during strafe and forward motion with gyro support
- stop safely on timeout or sensor failure

## What The Current Hardware Cannot Reliably Do

- confirm lateral free space in real time
- guarantee safe bypass in unknown cluttered scenes
- guarantee return to the original route center
- estimate absolute position after the bypass

Gyro only helps heading hold and recovery.
It does not provide absolute lateral position.

The front ultrasonic pair only confirms what is in front.
It does not confirm side clearance.

## Obstacle Logic

The final blocking decision is made on Arduino, not on Jetson.

Jetson provides one of these visual observations:

- `CLEAR`
- `PERSON LEFT`
- `PERSON CENTER`
- `PERSON RIGHT`
- `OBJECT LEFT`
- `OBJECT CENTER`
- `OBJECT RIGHT`

Arduino then combines that with the front ultrasonic pair:

- if front distance is close and vision says `PERSON`, obstacle type becomes `PERSON`
- if front distance is close and vision says `OBJECT`, obstacle type becomes `OBJECT`
- if front distance is close but vision is clear or stale, obstacle type becomes `UNKNOWN`

So generic obstacle handling is supported in two ways:

- YOLO sees a non-person object candidate and front ultrasonic confirms blocking
- YOLO misses or becomes stale, but front ultrasonic still confirms blocking, so Arduino treats it as `UNKNOWN`

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
OBS PERSON LEFT 82
OBS PERSON CENTER 76
OBS OBJECT RIGHT 64
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

- JSON telemetry lines
- state, action, front distances, heading, obstacle type, avoid side, and fault fields

## Jetson Runtime Notes

- current YOLO runtime depends on Docker on Jetson Nano
- host-side `venv` is still not finished
- current Docker test with `imshow` is about `8 FPS`
- `yolo26n.pt` and `yolo11n.pt` currently show similar FPS

Current optimization priority should be:

- code path reduction
- display overhead reduction
- runtime and pipeline tuning
- export/runtime changes such as TensorRT

It should not focus only on changing the model name.

## How To Run On Jetson

Run inside the current Jetson YOLO Docker runtime, not the unfinished host `venv`.

### Dependencies inside the runtime

- `ultralytics`
- `opencv-python`
- `pyserial`

If `pyserial` is missing:

```bash
pip3 install pyserial
```

### Normal run without display

```bash
cd software/jetson_yolo_tests/obstacle_avoidance_strafe_test
python3 obstacle_avoidance_strafe_bridge.py \
  --model yolo11n.pt \
  --serial /dev/ttyACM0 \
  --baud 115200 \
  --camera /dev/video0
```

### Auto-start Arduino test after serial is ready

```bash
python3 obstacle_avoidance_strafe_bridge.py \
  --model yolo11n.pt \
  --serial /dev/ttyACM0 \
  --baud 115200 \
  --camera /dev/video0 \
  --auto-start
```

### Optional display mode

Display is off by default because `imshow` adds overhead and current display-on testing is only around `8 FPS`.

```bash
python3 obstacle_avoidance_strafe_bridge.py \
  --model yolo11n.pt \
  --serial /dev/ttyACM0 \
  --camera /dev/video0 \
  --show
```

## Current Avoidance Policy

If the latched obstacle zone is:

- `LEFT`: Arduino defaults to strafe right
- `RIGHT`: Arduino defaults to strafe left
- `CENTER` or `NONE`: Arduino defaults to strafe right

This is a template-only rule.
It is not a real side-clearance decision.

## Current Limits And Risks

1. No side sensor means the robot cannot verify whether the chosen side is safe.
2. The fixed strafe time may under-shoot or over-shoot depending on floor friction and battery state.
3. The forward pass stage still depends on front distance becoming safe enough, which is only a partial proxy for successful bypass.
4. If the mecanum strafe itself is mechanically inconsistent, software alone cannot fully fix it.
5. Generic object detection is intentionally broad; it is useful for testing but not yet selective enough for production use.

## Suggested Improvement Direction

1. Tune per-motion PWM and strafe timing on the real robot.
2. Reduce Jetson display overhead and keep control in no-display mode.
3. Add a better runtime path such as TensorRT for YOLO.
4. If obstacle avoidance becomes mission-critical later, reintroduce lateral sensing or another reliable side-clearance method.

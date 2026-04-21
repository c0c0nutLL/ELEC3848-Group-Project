# final_code_optimized

Optimized Arduino sketch for the ELEC3848 required function.

## Deprecation Notice

This optimized branch is now a historical reference only.

Reason:

- it depends on the removed left/right side ultrasonic sensors
- it also reflects an older corridor-guidance assumption that is no longer used

Current hardware-valid straight-line work should use:

- gyro heading hold
- motor PWM trim / per-wheel compensation
- the remaining front ultrasonic pair only for docking / parking / near-range safety

Main sketch:
- `final_code_optimized.ino`

This version keeps the same project goal and overall path logic, but reorganizes the code into a clearer state machine and adds direction-specific compensation for the real robot chassis.

## Goal

The routine is designed to:
- start after a button press or serial command,
- approach the wall and build a stable reference pose,
- search for color and choose the required turn direction,
- turn 90 degrees with gyro feedback,
- approach the final parking area,
- use front and side ultrasonic sensors to correct the final pose,
- stop parallel to the wall and inside the parking box.

## Hardware Summary

### Drive motors

The sketch assumes a 4-wheel omnidirectional chassis with the following PWM and direction pins:

| Motor | PWM | DIR 1 | DIR 2 |
| --- | --- | --- | --- |
| A | 12 | 34 | 35 |
| B | 8 | 37 | 36 |
| C | 9 | 43 | 42 |
| D | 5 | A4 | A5 |

### Front ultrasonic sensors

| Position | Trig | Echo |
| --- | --- | --- |
| Front right | 6 | 4 |
| Front left | 10 | 7 |

### Side ultrasonic sensors

| Position | Trig | Echo |
| --- | --- | --- |
| Side left | 28 | 25 |
| Side right | 30 | 32 |

### Other peripherals

| Device | Pins / Notes |
| --- | --- |
| Start button | 33 |
| Color sensor OUT | 46 |
| Color sensor S0/S1/S2/S3 | 44 / 45 / 13 / 11 |
| Color sensor LED | 47 |
| MPU6050 | I2C, address `0x68` |
| OLED | I2C, reset disabled with `OLED_RESET = -1` |

## Routine Stages

The code is easiest to discuss by stage number instead of only state name.

| Stage | State | Purpose |
| --- | --- | --- |
| 1 | `TO27` | First fast forward approach |
| 2 | `A27ALN` | Midway body alignment using front ultrasonic diff |
| 3 | `TO27S` | Slow forward approach to the first stop |
| 4 | `CHK27` | First stop settle and front-distance recovery |
| 5 | `ALGN1` | Alignment before color search |
| 6 | `LEFT` / `RIGHT` | Strafe based on side ultrasonic space while searching color |
| 7 | `RECEN` | Recenter during color search every 2 seconds |
| 8 | `COLOR` | Color confirmation hold |
| 9 | `PREFWD` | Short forward preload before turning |
| 10 | `PRETRN` | Alignment before the 90-degree turn |
| 11 | `TURN` | Gyro-based 90-degree turn |
| 12 | `SHIFT` | Short right strafe bias after the turn |
| 13 | `TO9` | Final forward approach to the parking area |
| 13a | `AVDS` | Temporary outward strafe if the monitored side gets too close during stage 13 |
| 13b | `AVDT` | Extra 15-degree turn after `AVDS`, then return to stage 13 |
| 14 | `CHK9` | Final front-distance settle |
| 15 | `SIDE` | Final side-position adjustment using side ultrasonic sensor |
| 16 | `ALGN2` | Final front-distance check near 9 cm, then final parallel alignment |
| 17 | `DONE` | Final validation of side position, front average distance, and parallel alignment |
| 18 | `FAULT` | Error state |

## Core Logic

### First approach

- The robot starts with a fast forward segment.
- When the front average distance drops below `65 cm`, it pauses and aligns the body.
- It then continues forward more slowly until the first target distance near `27 cm`.
- After that, it stops briefly and rechecks the front distance.

### Color search

- The robot compares the left and right side ultrasonic readings.
- If the right side distance is larger, it strafes right.
- If the left side distance is larger, it strafes left.
- During this process, it performs a recenter step every 2 seconds.

### Turn decision

- Green means left turn.
- Red means right turn.
- The turn itself is closed-loop with gyro feedback.

### Final approach

- After the main 90-degree turn, the robot performs a short right strafe bias.
- During stage 13 (`TO9`), the robot keeps moving toward the wall while holding heading.
- If the monitored side gets too close:
  - left-turn route: monitor the right side sensor,
  - right-turn route: monitor the left side sensor,
  - if that side becomes smaller than `7 cm`, the routine temporarily interrupts stage 13,
  - it first strafes outward,
  - then turns an extra `15 degrees` in the route direction,
  - then returns to stage 13.

### Final side positioning

- After the final front-distance settle, the robot uses a side ultrasonic target:
  - left-turn route: right side target is `19.0 +/- 0.7 cm`,
  - right-turn route: left side target is `21.0 +/- 1.0 cm`.
- After side positioning, the robot enters stage 16.

### Final alignment and validation

- Stage 16 (`ALGN2`) does not only check parallel alignment.
- It first verifies that the front average distance is still close to `9 cm`.
- The current final distance check is `9.0 +/- 0.6 cm`.
- If the average distance has drifted, the robot corrects the front distance before continuing.
- Once the average distance is acceptable, it performs the final front-ultrasonic diff alignment.

- Stage 17 (`DONE`) is a validation state, not just a passive stop.
- In this state, the robot keeps checking:
  - side position target,
  - front average distance near `9 cm`,
  - final front diff alignment.
- If any of these checks fails, it automatically returns to the relevant correction stage:
  - side error -> `SIDE`,
  - average distance error -> `CHK9`,
  - front diff error -> `ALGN2`.
- Only when all final checks are satisfied does the robot remain stopped in `DONE`.

## Serial Commands

Open the serial monitor at `115200` baud.

Available commands:
- `G`: start the routine
- `X`: stop and return to idle
- `C`: recalibrate gyro bias and return to idle

The routine can also be started with the physical button on pin `33`.

## Telemetry Output

The serial monitor prints periodic telemetry including:
- current state,
- front right and left distances,
- front average and diff,
- side right and left distances,
- side average and diff,
- heading,
- gyro Z-axis angular velocity,
- raw and stable color result,
- fault message when present.

Example fields:

```text
ms=2993 state=TO9 right=161.8 left=168.8 avg=165.3 diff=-6.9 sideRight=42.1 sideLeft=55.7 sideAvg=48.9 sideDiff=-13.6 heading=0.0 gyroZ=0.00 rawColor=UNK stableColor=UNK
```

## OLED Output

The OLED uses a compact 4-line layout:

1. state and color
2. front ultrasonic right / left
3. side ultrasonic right / left
4. front average, front diff, heading

In `FAULT`, the OLED shows:
- `FAULT`,
- the fault message,
- front ultrasonic readings,
- side ultrasonic readings.

## Important Tuning Parameters

These constants are the main tuning entry points:

| Parameter | Meaning |
| --- | --- |
| `FIRST_TARGET_CM` | First stopping distance |
| `FIRST_STAGE_TRANSITION_CM` | Distance where the first fast approach switches to align |
| `SECOND_TARGET_CM` | Final front-distance target before side correction |
| `FINAL_DISTANCE_CHECK_TOLERANCE_CM` | Allowed error band around the final 9 cm average distance during stages 16 and 17 |
| `LEFT_TURN_SIDE_TARGET_CM` | Final side target for the left-turn route |
| `RIGHT_TURN_SIDE_TARGET_CM` | Final side target for the right-turn route |
| `APPROACH_SIDE_ESCAPE_TRIGGER_CM` | Side-distance trigger for temporary avoidance during stage 13 |
| `POSTTURN_BIAS_MS` / `POSTTURN_BIAS_PWM` | Fixed right strafe after the 90-degree turn |
| `LEFT_SIDE_TRIM` | Base left-side forward compensation |
| `FIRST_STAGE_LEFT_SIDE_TRIM` | Extra left-side trim during the first fast forward stage |
| `LEFT_AFTER_TURN_TRIM` | Forward trim used after the main turn |
| `STRAFE_LEFT_RF_TRIM` | Left-strafe extra compensation on the right-front wheel |
| `STRAFE_RIGHT_RF_TRIM` | Right-strafe extra compensation on the right-front wheel |
| `STRAFE_RIGHT_LR_TRIM` | Right-strafe extra compensation on the left-rear wheel |

## Fault Conditions

The sketch enters `FAULT` when a critical condition occurs, including:
- ultrasonic timeout,
- side ultrasonic timeout,
- MPU6050 not ready or repeated read timeout,
- final wall distance becoming too small,
- 90-degree turn timeout,
- final avoidance turn timeout,
- timeout during final validation distance sampling,
- timeout during final validation side-distance sampling.

## Notes

- The code assumes real chassis asymmetry. Left and right motion do not share exactly the same compensation.
- The first approach, final approach, strafing, and turning all use different compensation values.
- `OLED_RESET` is disabled because pin `28` is now used by the left side ultrasonic trigger.
- This sketch is intended as the optimized branch of the required-function code, not as a minimal test sketch.

# Full Mission Flow Sandbox

This directory contains a single Arduino-side sandbox sketch for the current planned end-to-end mission flow:

- `FULL_MISSION`
- `PICKUP_MISSION`
- `DELIVERY_MISSION`

What this sketch does:

- keeps the frozen mission-stage names from `plan.md`
- includes pickup-side arm sequence, door open/load/close, turnaround, delivery-side unlock wait, and unload door-close timing
- models the current obstacle rule: obstacle handling triggers when obstacle detection is present and the front distance is below the threshold
- supports both serial-command simulation and a local close button input
- drives two task-status LEDs directly from the Arduino-side mission runtime

What this sketch does **not** do yet:

- real RFID register reads are not integrated here yet
- real QR verification still belongs to the Jetson side; Arduino only waits for `OPEN_DOOR`
- final motor PWM, servo angles, door-servo pin, and exact travel timings are still assumed values

Useful serial commands:

- `START FULL`
- `START PICKUP`
- `START DELIVERY`
- `RFID PICKUP`
- `RFID DELIVERY`
- `OPEN_DOOR`
- `CLOSE_DOOR`
- `OBSTACLE PERSON 30`
- `OBSTACLE OBJECT 30`
- `OBSTACLE CLEAR`
- `DIST 40`
- `STATUS`
- `ABORT`

## Task Status LEDs

This sketch is currently the closest Arduino-side full-mission sandbox, so the LED logic is attached here first.

LED pins:

- red LED: `pin 41`
- green LED: `pin 47`

LED meaning:

- red LED turns on when `obstacleRoutineActive == true`
  - this covers the whole obstacle interrupt routine, including stop-hold, strafe, pass, and return
  - it turns off immediately after the obstacle routine exits
- green LED turns on when `currentStage == STAGE_WAIT_USER_UNLOCK`
  - this means the robot has already reached the delivery-side wait-for-user-unlock stage
  - it turns off as soon as the system leaves `STAGE_WAIT_USER_UNLOCK`, including after `OPEN_DOOR`

Implementation note:

- LED output is centralized through `updateTaskStatusLeds()`
- the sketch does not scatter raw LED `digitalWrite()` calls across stage branches
- `STATUS` telemetry now also includes:
  - `obstacle_routine_active`
  - `obstacle_routine_step`
  - `red_led`
  - `green_led`

The exact assumed values that still need hardware confirmation are tracked separately in `ASSUMED_VALUES_AND_CHANGE_POINTS.md`.

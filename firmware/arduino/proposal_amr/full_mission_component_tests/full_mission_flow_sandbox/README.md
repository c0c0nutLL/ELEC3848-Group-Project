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

The exact assumed values that still need hardware confirmation are tracked separately in `ASSUMED_VALUES_AND_CHANGE_POINTS.md`.

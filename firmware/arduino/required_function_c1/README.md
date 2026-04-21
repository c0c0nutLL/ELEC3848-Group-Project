# Required Function C1

This folder stores Arduino code for the required parking function.

## Historical Note

Some older sketches in this folder were created for experiments that are no
longer part of the current hardware baseline.

- Side ultrasonic experiments are now historical only.
- Current robot hardware keeps the front ultrasonic pair for docking / parking.
- Current straight-line work should use gyro heading hold plus motor PWM trim.

## Scope

- start 1 meter from the wall
- move into the parking zone
- stop 8 cm from the wall
- align parallel to the wall
- avoid touching the wall during alignment
- send sensor data to OLED and computer

## Expected Inputs

- ultrasonic sensor data
- gyroscope data
- light tracker data
- bluetooth control or trigger if needed

## Planned Files

- `required_function_c1.ino`
- motor control helpers
- sensor reading helpers
- OLED display helpers
- serial telemetry helpers

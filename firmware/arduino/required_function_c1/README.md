# Required Function C1

This folder stores Arduino code for the required parking function.

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

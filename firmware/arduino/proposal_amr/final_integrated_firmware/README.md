# Final Integrated Firmware

This directory is the authoritative Arduino runtime for the proposal-stage AMR system.

Authoritative sketch:

- `final_integrated_firmware.ino`

This sketch replaces the previous split between a business-flow sandbox and a separate interrupt/resume demo. The final structure is:

- one business mission state machine
- one run-mode layer wrapped around that mission state machine

## Ownership Split

Arduino owns:

- mission and task state
- run mode (`AUTO`, `MANUAL_OVERRIDE`, `FAULT`)
- motor, arm, and cargo-door control
- RFID reading and RFID station matching
- telemetry JSON sent to Jetson

Jetson owns:

- web UI
- HTTP APIs
- LAN manual-access password gate
- unlock password flow
- serial bridging between browser and Arduino

## Canonical Tasks

- `FULL_MISSION`
- `PICKUP_MISSION`
- `DELIVERY_MISSION`
- `MANUAL_TEST`

## Canonical Stages

- `IDLE`
- `START`
- `TRAVEL_TO_PICKUP`
- `VERIFY_PICKUP_STATION`
- `PICKUP_SEQUENCE`
- `OPEN_DOOR_FOR_LOAD`
- `LOAD_CARGO`
- `CLOSE_DOOR_AFTER_LOAD`
- `TURN_AROUND`
- `TRAVEL_TO_DELIVERY`
- `VERIFY_DELIVERY_STATION`
- `WAIT_USER_UNLOCK`
- `OPEN_DOOR_FOR_UNLOAD`
- `WAIT_DOOR_CLOSE`
- `AUTO_CLOSE_DOOR`
- `REPORT_STATUS`
- `DONE`
- `FAULT`

Resume is stage-level. Sequence stages may restart from a safe stage entry instead of resuming in the middle of a servo move.

## Serial Command Protocol

Task and mode commands:

- `TASK START FULL_MISSION`
- `TASK START PICKUP_MISSION`
- `TASK START DELIVERY_MISSION`
- `TASK START MANUAL_TEST`
- `TASK ABORT`
- `OVERRIDE TAKE`
- `OVERRIDE RESUME`
- `OVERRIDE ABORT`

Drive and arm commands:

- `DRIVE FORWARD <PWM>`
- `DRIVE BACKWARD <PWM>`
- `DRIVE LEFT <PWM>`
- `DRIVE RIGHT <PWM>`
- `DRIVE TURN_LEFT <PWM>`
- `DRIVE TURN_RIGHT <PWM>`
- `DRIVE STOP <PWM>`
- `ARM LIFT <ANGLE>`
- `ARM GRIP <ANGLE>`

Door commands:

- `OPEN_DOOR`
- `CLOSE_DOOR`

Legacy `START FULL`, `START PICKUP`, `START DELIVERY`, and `ABORT` remain accepted for compatibility where practical.

## Telemetry

The sketch emits structured JSON over serial. Important fields include:

- `mode_protocol_version`
- `current_mode`
- `current_auto_stage`
- `saved_auto_stage`
- `resume_pending`
- `manual_session_active`
- `task_active`
- `fault_latched`
- `resume_note`
- `status_note`
- `selected_task`
- `cargo_door`
- `door_auto_close_ms`
- `pickup_rfid_matched`
- `delivery_rfid_matched`
- `last_rfid_station`
- `rfid_tag`
- `rfid_match`
- `obstacle_routine_active`
- `obstacle_routine_step`
- `red_led`
- `green_led`
- `control_direction`
- `motor_pwm`
- `arm_lift_deg`
- `arm_grip_deg`
- `right_cm`
- `left_cm`
- `avg_cm`
- `diff_cm`
- `heading_deg`

`right_cm` and `left_cm` are the current front ultrasonic pair readings. `avg_cm` and `diff_cm` are derived from those readings.

## Pin Mapping

Pin assignments were updated against `hardware/amr_pin_mapping_template.xlsx`.

Applied workbook-derived values in the current sketch include:

- front ultrasonic trigger and echo pins
- arm lift pin
- arm gripper pin
- door servo pin
- RFID SS pin
- RFID RST pin
- red LED pin
- green LED pin

## Source Inputs

The integrated runtime was assembled primarily from:

- `full_mission_component_tests/full_mission_flow_sandbox/full_mission_flow_sandbox.ino`
- `interrupt_resume_demo/interrupt_resume_demo.ino`
- `firmware/arduino/HW-868/HW-868.ino`

Historical sketches remain useful as references, but this directory is now the single obvious runtime entry point for the proposal-stage robot.

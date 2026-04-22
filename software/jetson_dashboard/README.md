# Jetson Dashboard

This directory is the Jetson Nano web and serial bridge for the AMR project.

Jetson responsibilities:

- serve the browser UI
- expose HTTP APIs
- enforce the LAN manual-access password gate
- handle the unlock page and password flow
- forward browser commands to Arduino over USB serial
- mirror Arduino telemetry to browser clients

Jetson is not the mission source of truth. Arduino owns task, mode, stage, door, RFID, and actuator state.

## Authoritative Files

- `server.py`
  - main Jetson service
  - reads Arduino JSON telemetry
  - exposes task, mode, control, arm, unlock, and task-selection APIs
- `tasks.html`
  - real task-selection page
  - starts and aborts canonical Arduino tasks
- `manual.html`
  - manual drive and arm page
  - respects Arduino `MANUAL_OVERRIDE` gating
- `status.html`
  - telemetry-backed live status view
- `debug.html`
  - telemetry-backed raw JSON and event log view
- `home.html`
  - top-level navigation page
- `unlock.html`
  - phone/browser unlock flow

## Paired Arduino Runtime

Use this sketch as the authoritative paired runtime:

- `firmware/arduino/proposal_amr/final_integrated_firmware/final_integrated_firmware.ino`

Historical sketches under `proposal_amr/` remain as references only. They are not the authoritative runtime path anymore.

## Task Model

Canonical task ids:

- `FULL_MISSION`
- `PICKUP_MISSION`
- `DELIVERY_MISSION`
- `MANUAL_TEST`

Task-selection APIs are now real:

- `GET /api/task-selection/state`
- `POST /api/task-selection/select`
- `POST /api/task-selection/start`
- `POST /api/task-selection/stop`
- `POST /api/task-selection/resume`

`/api/task-selection/state` is backed by Arduino telemetry plus a small Jetson-side selection hint when no `selected_task` has arrived yet.

Start commands sent over serial:

- `TASK START FULL_MISSION`
- `TASK START PICKUP_MISSION`
- `TASK START DELIVERY_MISSION`
- `TASK START MANUAL_TEST`

Other runtime commands sent over serial:

- `TASK ABORT`
- `OVERRIDE TAKE`
- `OVERRIDE RESUME`
- `OVERRIDE ABORT`
- `DRIVE FORWARD <PWM>`
- `DRIVE BACKWARD <PWM>`
- `DRIVE LEFT <PWM>`
- `DRIVE RIGHT <PWM>`
- `DRIVE TURN_LEFT <PWM>`
- `DRIVE TURN_RIGHT <PWM>`
- `DRIVE STOP <PWM>`
- `ARM LIFT <ANGLE>`
- `ARM GRIP <ANGLE>`
- `OPEN_DOOR`
- `CLOSE_DOOR`

## Runtime State

Expected Arduino mode layer:

- `AUTO`
- `MANUAL_OVERRIDE`
- `FAULT`

Expected business stages include:

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

The Jetson UI does not maintain a second autonomous state machine. It renders Arduino state.

## Telemetry Expectations

Important fields consumed by the Jetson pages include:

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

`right_cm`, `left_cm`, `avg_cm`, and `diff_cm` are treated as the current front ultrasonic pair measurements and their derived values.

## Unlock Flow

Unlock remains on Jetson:

- browser submits password to Jetson
- Jetson verifies the password
- Jetson sends `OPEN_DOOR`
- Jetson can later send `CLOSE_DOOR`

Arduino does not implement the web password flow. It only reacts to the door commands and reports door state in telemetry.

## Mock Mode

`--mock` is still supported for UI bring-up and server testing. In real serial mode, task pages and status pages are expected to follow Arduino telemetry.

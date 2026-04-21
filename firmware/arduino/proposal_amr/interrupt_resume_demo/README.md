# Interrupt / Resume Demo

This directory contains a proposal-stage Arduino demo sketch that adds a real manual override path on top of a minimal auto task flow.

Main sketch:

- [interrupt_resume_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino)

## What This Demo Does

The sketch implements two layers of runtime state:

- `current_mode`
  - `AUTO`
  - `MANUAL_OVERRIDE`
  - `FAULT`
- `current_auto_stage`
  - `IDLE`
  - `START`
  - `CORRIDOR_DRIVE`
  - `APPROACH_STATION`
  - `DOCK_AND_ALIGN`
  - `VERIFY_STATION`
  - `ARM_ACTION`
  - `REPORT_STATUS`
  - `DONE`
  - `FAULT`

The intent is:

1. Jetson starts a proposal-style auto task.
2. If the operator clicks `Take Manual Control`, the Arduino freezes auto progress and enters `MANUAL_OVERRIDE`.
3. Manual chassis and arm commands are accepted only in `MANUAL_OVERRIDE`.
4. If the operator clicks `Return To Auto / Resume Task`, the sketch resumes from the saved stage entry point.
5. If the operator clicks `Cancel Resume / Abort Task`, the saved task context is cleared and the robot stays in manual mode.

## Important Scope Boundary

This is not the final proposal mission state machine.

It is a minimal runnable proposal auto-flow demo attached to the current Jetson dashboard so the interrupt / resume chain can be tested honestly end-to-end before the final mission logic exists.

Current auto flow:

- `START`
- `CORRIDOR_DRIVE`
- `APPROACH_STATION`
- `DOCK_AND_ALIGN`
- `VERIFY_STATION`
- `ARM_ACTION`
- `REPORT_STATUS`
- `DONE`

Notes:

- station verification is currently simulated by a timed stage and telemetry fields
- the arm sequence is currently a fixed safe demo sequence
- the front ultrasonic pair is used for approach / dock / align
- there is no map, localization, or mid-motion positional recovery

## Resume Semantics

Resume is stage-level, not millisecond-level.

Safe resume points:

- `START`
- `CORRIDOR_DRIVE`
- `APPROACH_STATION`
- `DOCK_AND_ALIGN`
- `VERIFY_STATION`
- `ARM_ACTION`
- `REPORT_STATUS`

Important rule:

- `ARM_ACTION` does not resume from the middle of a servo move
- it restarts from the safe entry of the arm sequence

No saved resume point:

- `IDLE`
- `DONE`
- `FAULT`

## Serial Command Protocol

Jetson -> Arduino commands:

- `TASK START`
- `TASK ABORT`
- `OVERRIDE TAKE`
- `OVERRIDE RESUME`
- `OVERRIDE ABORT`
- `DRIVE FORWARD <0-255>`
- `DRIVE BACKWARD <0-255>`
- `DRIVE LEFT <0-255>`
- `DRIVE RIGHT <0-255>`
- `DRIVE STOP <0-255>`
- `ARM LIFT <0-180>`
- `ARM GRIP <0-180>`
- `GYRO`
- `OPEN_DOOR`

Arduino -> Jetson telemetry adds:

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

## Safety / Failure Rules

This demo intentionally blocks fake resume:

- if `fault_latched == true`, normal resume is rejected
- if there is no saved stage, resume is rejected
- manual drive commands are rejected unless `current_mode == MANUAL_OVERRIDE`
- manual arm commands are rejected unless `current_mode == MANUAL_OVERRIDE`

Current fault entry examples:

- repeated MPU6050 read failure during auto
- MPU6050 missing when auto start is requested
- repeated invalid front-distance readings during approach / dock stages

## How To Use

1. Flash [interrupt_resume_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino) to the Arduino Mega.
2. Start the Jetson dashboard service from [software/jetson_dashboard](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard).
3. Open the dashboard in a browser.
4. Click `Start Auto Task`.
5. During any active auto stage, click `Take Manual Control`.
6. Use the drive pad and arm controls while the mode is `MANUAL_OVERRIDE`.
7. Click `Return To Auto / Resume Task` to resume from the saved stage entry point.
8. Click `Cancel Resume / Abort Task` if you want to stay in manual mode and drop the saved task.
9. If manual mode no longer has a saved task, `Start Auto Task` is allowed again and will leave manual mode by starting a fresh auto run.

## Current Limitations

- This demo is a proposal-stage integration target, not the final mission.
- The verification stage is still a timed placeholder, not a real station sensor stack.
- The arm action is a fixed sequence, not a task planner.
- Resume is semantic-stage based only.
- If the chassis was physically displaced during manual override, the resumed stage is only as good as the current robot pose.
- Because there is no localization layer, recovery is intentionally limited to stage re-entry.

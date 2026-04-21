# Agent Memory

Last updated: 2026-04-21

This file is the long-term memory handoff for future threads working in this repo.
It summarizes the current project state, active plan, important files, hardware decisions, and coordination rules.

## 1. Current Project Phase

The `required_function_c1` phase is treated as completed baseline work.

What future threads should focus on now:

- the active project plan is the root-level `plan.md`
- the main target is the proposal-stage delivery AMR implementation
- `required_function_c1` should be used as a technical reference, not as the main target

Meaning:

- do not continue planning against `firmware/arduino/required_function_c1/PLAN.md` unless the user explicitly wants Required Function maintenance
- use the root `plan.md` as the current source of truth for overall project progress

## 2. Active Plan Files

### Main active plan

- [plan.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/plan.md)
  - current project-wide implementation plan
  - reflects the latest removal of side ultrasonics

### Historical phase plan

- [firmware/arduino/required_function_c1/PLAN.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/PLAN.md)
  - historical Required Function phase plan
  - useful for reference only

## 3. Current Project Direction

The project is an indoor smart delivery AMR.

The intended workflow is:

`target selection -> gyro-assisted straight travel -> docking -> station verification -> cargo pickup or release -> telemetry upload -> dashboard display and task log`

Compared with the original proposal, the real implementation now includes:

- a two-servo mechanical arm
- gyro-assisted straight motion using per-wheel PWM trim

Important scope updates:

- the planned 3-sensor black/white line-following branch is cancelled
- the two side-mounted ultrasonic sensors are also removed from the current hardware plan
- straight-line control must not rely on corridor wall-distance balancing anymore
- the front ultrasonic pair is still active for docking / parking / close-range safety

Current planned UI task list:

1. Full mission
2. Obstacle-avoidance mission
3. Pickup mission
4. Delivery mission
5. Manual test

Current intended full-mission flow:

`START -> STRAIGHT_TRAVEL_TO_PICKUP -> RFID_PICKUP_DETECTED -> PICKUP_SEQUENCE -> OPEN_BOX -> LOAD_ITEM -> CLOSE_BOX -> TURN_AROUND -> STRAIGHT_TRAVEL_TO_DELIVERY -> RFID_DELIVERY_DETECTED -> WAIT_USER_UNLOCK -> USER_OPEN_DOOR -> USER_CLOSE_DOOR or AUTO_CLOSE_AFTER_60S -> REPORT_STATUS -> DONE`

Important mission rule:

- obstacle avoidance may interrupt almost any active mission stage
- after obstacle handling, the system should resume the interrupted task stage or the nearest safe recovery point

## 4. Important Files

### Project definition and planning

- [ELEC3848 Proposal.pdf](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/ELEC3848%20Proposal.pdf)
  - original proposal scope and six-module workflow

- [plan.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/plan.md)
  - current main project plan

- [agent.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/agent.md)
  - this memory handoff file

### Required Function baseline

- [firmware/arduino/required_function_c1/final_code/final_code.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/final_code/final_code.ino)
  - integrated Required Function implementation
  - keeps valid front-ultrasonic docking logic

- [firmware/arduino/required_function_c1/motion_ultrasonic_parking_test/motion_ultrasonic_parking_test.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/motion_ultrasonic_parking_test/motion_ultrasonic_parking_test.ino)
  - front ultrasonic parking / stopping test
  - still relevant

- [firmware/arduino/required_function_c1/straight_line_power_test/straight_line_power_test.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/straight_line_power_test/straight_line_power_test.ino)
  - active gyro + four-wheel PWM straight-line tuning sketch

### Proposal firmware draft

- [firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino)
  - proposal-stage telemetry bridge
  - supports Jetson manual control commands
  - now uses gyro heading hold + per-wheel trim
  - still keeps front ultrasonic telemetry fields

- [firmware/arduino/proposal_amr/motor_calibration_assistant/motor_calibration_assistant.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/motor_calibration_assistant/motor_calibration_assistant.ino)
  - active calibration utility
  - no side ultrasonic dependency
  - forward / backward auto search is gyro-based
  - strafe search is explicitly semi-automatic

- [firmware/arduino/proposal_amr/motor_calibration_assistant/README.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/motor_calibration_assistant/README.md)
  - operational notes for the calibration utility

### Interrupt / Resume Demo

- [firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino)
  - current proposal-stage interrupt / resume integration demo
  - adds task-stage telemetry for the Jetson dashboard
  - is the current target for UI manual takeover testing

- [firmware/arduino/proposal_amr/interrupt_resume_demo/README.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/interrupt_resume_demo/README.md)
  - documents stage-level resume rules and serial protocol

### Jetson Nano side

- [software/jetson_dashboard/server.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/server.py)
  - main Jetson service
  - reads telemetry from Arduino
  - serves dashboard and unlock pages
  - sends serial commands back to Arduino

- [software/jetson_dashboard/dashboard.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/dashboard.html)
  - browser dashboard page

- [software/jetson_dashboard/unlock.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock.html)
  - fixed-password unlock page

- [software/jetson_dashboard/unlock_config.json](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock_config.json)
  - unlock configuration

### Jetson YOLO test area

- [software/jetson_yolo_tests/README.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_yolo_tests/README.md)
  - usage notes and environment notes for Jetson-side YOLO testing

- [software/jetson_yolo_tests/camera_detection_test.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_yolo_tests/camera_detection_test.py)
  - live camera detection test

- [software/jetson_yolo_tests/image_detection_test.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_yolo_tests/image_detection_test.py)
  - single-image detection test

### Hardware reference and pin mapping

- [hardware/amr_pin_mapping_template.xlsx](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/hardware/amr_pin_mapping_template.xlsx)
  - hardware pin mapping workbook
  - should remain the hardware source of truth

- [firmware/arduino/singleServo/singleServo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/singleServo/singleServo.ino)
  - servo vendor sample
  - useful for arm-related sketches

## 5. Project Structure

Current practical structure:

```text
.
|-- agent.md
|-- plan.md
|-- README.md
|-- firmware/
|   `-- arduino/
|       |-- proposal_amr/
|       |   |-- telemetry_serial_demo/
|       |   |-- motor_calibration_assistant/
|       |   |-- interrupt_resume_demo/
|       |   `-- obstacle_avoidance_test/        (historical / obsolete)
|       |-- required_function_c1/
|       |   |-- final_code/
|       |   |-- final_code_optimized/           (historical / obsolete)
|       |   |-- straight_line_power_test/
|       |   |-- motion_ultrasonic_parking_test/
|       |   `-- dual_ultrasonic_oled_test/      (historical / obsolete)
|       |-- singleServo/
|       `-- motortest/
|-- hardware/
|   `-- amr_pin_mapping_template.xlsx
|-- software/
|   |-- jetson_dashboard/
|   `-- jetson_yolo_tests/
|-- reflection/
|-- Test_Code_24A/
`-- 装配/
```

Interpretation:

- `firmware/arduino/proposal_amr/`
  - active Arduino-side implementation area for the proposal stage

- `firmware/arduino/required_function_c1/`
  - completed baseline and reference area

- directories explicitly marked `historical / obsolete`
  - kept for reference only
  - should not be treated as current hardware targets

## 6. Important Hardware Memory

### Mechanical arm

The arm has two servos:

- gripper open/close servo: `pin 44`
- arm up/down servo: `pin 45`

### Navigation hardware decision

Current navigation planning uses:

- MPU6050 gyroscope
- per-wheel PWM tuning / trim
- front ultrasonic pair for docking / parking / short-range safety

### Interrupt / Resume Integration

There is still no final proposal mission state machine in `proposal_amr`.

Current practical target for task-level interrupt / resume work:

- `firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino`

Interpretation:

- this sketch is the current contract for Jetson UI manual takeover testing
- it separates `current_mode` from `current_auto_stage`
- resume is stage-level and may restart from a safe stage entry instead of the exact moment of interruption

Current hardware does not use:

- left-side ultrasonic
- right-side ultrasonic
- color-sensor line following

### Calibration reality

Future threads must remember:

- forward / backward straightness can be improved automatically with gyro-based search
- strafe parameters can only be semi-automatic without encoder or lateral position sensing
- no thread should describe the current system as “fully auto calibrated” for every motion mode

## 7. Communication Architecture

Current intended data path:

`Arduino Mega -> USB serial -> Jetson Nano -> browser dashboard`

Control can also go back the other way:

`browser / unlock page -> Jetson Nano -> USB serial -> Arduino Mega`

Important implications:

- Arduino is the real-time low-level controller
- Jetson is the network / dashboard / logging / unlock side
- Jetson is also the current vision runtime side for YOLO experiments
- USB serial is the current bridge
- telemetry schema and command schema should stay stable across both sides

## 8. What Is Already Working

These things should be treated as available baseline capability:

- Required Function parking and alignment reference code
- front ultrasonic + gyro + OLED integration experience
- Jetson dashboard draft
- Jetson unlock page with fixed password
- Arduino-to-Jetson JSON telemetry draft
- Jetson-to-Arduino `OPEN_DOOR` command path
- Jetson `--mock` mode for browser-side testing
- Jetson Docker-based Ultralytics YOLO runtime
- active gyro-based straight-line tuning and calibration utilities

## 9. What Is Not Finished

These are the main unfinished areas:

1. final proposal Arduino mission state machine
2. real integrated RFID verification
3. real integrated arm behavior for pickup / release
4. integration of long-range gyro-based straight motion with final docking
5. final mission dashboard and task logs
6. final decision on whether YOLO obstacle handling belongs in the main mission

## 10. Immediate Priorities

The current recommended work order is:

1. keep hardware, code, and docs aligned with the “no side ultrasonic” decision
2. continue tuning gyro + per-wheel PWM profiles
3. define the final proposal Arduino telemetry and state names
4. create the first integrated proposal mission sketch under `proposal_amr`
5. merge docking, verification, arm action, telemetry, and Jetson UI
6. keep YOLO obstacle handling as a separate test path until latency and behavior are validated

## 11. Coordination Rules For Other Threads

- treat [plan.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/plan.md) as the active master plan
- treat [agent.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/agent.md) as thread memory / handoff notes
- treat `required_function_c1` as reference baseline, not the active target
- do not assume side ultrasonics are still installed
- keep the distinction clear:
  - front ultrasonic pair: active
  - side ultrasonics: removed / obsolete
- do not mark a subsystem as complete unless it is integrated into the proposal-stage workflow
- if telemetry field names change, update both Arduino and Jetson side together
- if hardware assumptions change, update:
  - `hardware/amr_pin_mapping_template.xlsx`
  - the relevant Arduino sketches
  - `plan.md` and `agent.md`

## 12. Useful Reminders

- Jetson Nano Python version is `3.6.9`
- `server.py` was already adjusted for Python 3.6 compatibility
- the unlock system currently uses a fixed password and may later move to another mode
- the final role of QR unlock is still undecided
- Jetson-side Ultralytics YOLO currently works through Docker
- host-side YOLO `venv` setup is still incomplete
- future YOLO improvement should first look at code/runtime optimization, not assume model-name changes alone will help

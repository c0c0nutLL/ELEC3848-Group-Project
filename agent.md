# Agent Memory

Last updated: 2026-04-17

This file is the long-term memory handoff for future threads working in this repo.
It summarizes the current project state, active plan, important files, project structure, hardware changes, and coordination rules.

## 1. Current Project Phase

The `required_function_c1` phase is treated as completed baseline work.

What future threads should focus on now:

- the active project plan is the root-level `plan.md`
- the main target is the proposal-stage delivery AMR implementation
- `required_function_c1` should be used as a technical reference, not as the main project target

Meaning:

- do not continue planning against `firmware/arduino/required_function_c1/PLAN.md` unless the user explicitly wants Required Function maintenance
- use the root `plan.md` as the current source of truth for overall project progress

## 2. Active Plan Files

### Main active plan

- [plan.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/plan.md)
  - this is the current project-wide implementation plan
  - it merges:
    - original proposal goals
    - already completed Required Function baseline
    - Jetson Nano telemetry / dashboard / unlock work
    - newly added arm subsystem
    - latest navigation decision: no line-following color sensors

### Historical phase plan

- [firmware/arduino/required_function_c1/PLAN.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/PLAN.md)
  - this is the historical plan for the Required Function phase
  - useful for reference only
  - not the main planning file anymore

## 3. Current Project Direction

The project is an indoor smart delivery AMR.

The intended final workflow is:

`target selection -> straight-line corridor movement -> docking -> station verification -> cargo pickup or release -> telemetry upload -> dashboard display and task log`

Compared with the original proposal, the real implementation now includes:

- a two-servo mechanical arm
- corridor-style straight-line control using side ultrasonics + gyro

So the practical subsystem list is now:

1. base motion control
2. corridor straight-line control with side ultrasonics + gyro
3. docking and close-range safety with ultrasonic + gyro
4. station verification, originally planned as RFID
5. arm manipulation
6. cargo handling / unlock interaction
7. telemetry bridge Arduino <-> Jetson
8. dashboard and task logs

Important scope update:

- previously discussed 3-sensor black/white line-following is cancelled
- the extra color sensors will not be installed
- do not keep planning around route-line detection

## 4. Important Files

### Project definition and planning

- [ELEC3848 Proposal.pdf](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/ELEC3848%20Proposal.pdf)
  - original proposal scope and six-module workflow

- [plan.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/plan.md)
  - current main project plan

- [agent.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/agent.md)
  - this thread memory file

### Required Function baseline

- [firmware/arduino/required_function_c1/final_code/final_code.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/final_code/final_code.ino)
  - integrated Required Function implementation
  - contains motion, docking, OLED, telemetry, and state machine logic

- [firmware/arduino/required_function_c1/final_code_optimized/final_code_optimized.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/final_code_optimized/final_code_optimized.ino)
  - optimized / iterated version of the Required Function integrated code

- [firmware/arduino/required_function_c1/README.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/README.md)
  - overview of the Required Function code area

### Proposal firmware draft

- [firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino)
  - current proposal-stage Arduino draft
  - sends JSON telemetry to Jetson over USB serial
  - supports `OPEN_DOOR` command from Jetson
  - currently a communication / integration draft, not the final mission state machine

### Jetson Nano side

- [software/jetson_dashboard/server.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/server.py)
  - main Jetson service
  - reads telemetry from Arduino
  - serves dashboard and unlock pages
  - sends serial commands back to Arduino
  - made compatible with Python `3.6.9`

- [software/jetson_dashboard/dashboard.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/dashboard.html)
  - browser dashboard page

- [software/jetson_dashboard/unlock.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock.html)
  - fixed-password unlock page for phone QR workflow

- [software/jetson_dashboard/unlock_config.json](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock_config.json)
  - unlock configuration
  - currently fixed-password mode

- [software/jetson_dashboard/requirements.txt](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/requirements.txt)
  - Python dependency list

### Hardware reference and pin mapping

- [hardware/amr_pin_mapping_template.xlsx](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/hardware/amr_pin_mapping_template.xlsx)
  - current hardware pin mapping workbook
  - should be treated as the hardware source of truth

- [firmware/arduino/singleServo/singleServo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/singleServo/singleServo.ino)
  - sample code from the servo vendor
  - should be used as a reference when writing new arm servo test code

### Older reference code

- [Test_Code_24A/Test Code/Car_Volt_Feedback_24A/Car_Volt_Feedback_24A.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/Test_Code_24A/Test%20Code/Car_Volt_Feedback_24A/Car_Volt_Feedback_24A.ino)
  - older low-level reference code for motors, OLED, servo, Bluetooth, and battery feedback
  - useful as reference only

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
|       |   `-- telemetry_serial_demo/
|       |-- required_function_c1/
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
  - should become the main Arduino-side implementation area for the proposal stage

- `firmware/arduino/required_function_c1/`
  - completed baseline and reference area

- `software/jetson_dashboard/`
  - current Jetson-side integration area
  - active and important

- `software/jetson_yolo_tests/`
  - side capability / preparation area
  - not yet part of the core mission flow

- `reflection/`
  - presentation / reflection materials
  - not the main implementation area

- `Test_Code_24A/`
  - older vendor / course sample and reference material

## 6. Important Hardware Memory

### Mechanical arm

New arm has two servos:

- gripper open/close servo: `pin 44`
- arm up/down servo: `pin 45`

This is a major change.

### Navigation hardware decision

Current navigation planning uses:

- left-side ultrasonic
- right-side ultrasonic
- MPU6050 gyroscope

Working assumption:

- the robot moves between roughly parallel side walls
- corridor width is approximately constant over the controlled segment
- straight-line control comes from side-distance balance plus heading hold

### Cancelled line-following branch

Future threads must remember:

- the planned 3 front-mounted color sensors are cancelled
- do not allocate time to black/white path-line following unless the user explicitly reopens that direction
- old color-sensor tests under `required_function_c1` remain only as historical experiments

## 7. Communication Architecture

Current intended data path:

`Arduino Mega -> USB serial -> Jetson Nano -> browser dashboard`

Control can also go back the other way:

`browser / unlock page -> Jetson Nano -> USB serial -> Arduino Mega`

Important implications:

- Arduino is the real-time low-level controller
- Jetson is the network / dashboard / logging / unlock side
- USB serial is the current bridge
- telemetry schema and command schema should stay stable across both sides

## 8. What Is Already Working

These things should be treated as already available baseline capability:

- Required Function parking and alignment reference code
- ultrasonic + gyro + OLED integration experience
- Jetson dashboard draft
- Jetson unlock page with fixed password
- Arduino-to-Jetson JSON telemetry draft
- Jetson-to-Arduino `OPEN_DOOR` command path
- Jetson `--mock` mode for browser-side testing

## 9. What Is Not Finished

These are the main unfinished areas:

1. final proposal Arduino mission state machine
2. real integrated RFID verification
3. real integrated arm behavior for pickup / release
4. straight-line corridor control using side ultrasonics + gyro
5. integration of long-range corridor movement with close-range docking
6. final mission dashboard and task logs

## 10. Immediate Priorities

The current recommended work order is:

1. update the hardware pin mapping for the arm and the side ultrasonics
2. write arm servo test sketches using `singleServo.ino` as reference
3. build the first straight-line corridor control sketch using side ultrasonics + gyro
4. define the final proposal Arduino telemetry and state names
5. create the first integrated proposal mission sketch under `proposal_amr`
6. merge docking, verification, arm action, telemetry, and Jetson UI

## 11. Coordination Rules For Other Threads

- treat [plan.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/plan.md) as the active master plan
- treat [agent.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/agent.md) as thread memory / handoff notes
- treat `required_function_c1` as reference baseline, not the active target
- do not mark a subsystem as complete unless it is integrated into the proposal-stage workflow
- if telemetry field names change, update both Arduino and Jetson side together
- if hardware assumptions change, update:
  - `hardware/amr_pin_mapping_template.xlsx`
  - the relevant Arduino sketches
  - `plan.md` if the system design changes

## 12. Useful Reminders

- Jetson Nano Python version is `3.6.9`
- `server.py` was already adjusted for Python 3.6 compatibility
- the unlock system currently uses a fixed password and may later move to another mode
- the final role of QR unlock is still undecided
- YOLO exists as a side preparation area, not core mission logic yet

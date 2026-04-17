# ELEC3848 Project Plan

Last updated: 2026-04-17

This file is the current project-level source of truth.
It merges:

- the original proposal workflow
- the already completed Required Function work
- the current Jetson Nano telemetry and unlock work
- the newly added arm hardware
- the latest navigation decision: no line-following color sensors

## 1. Project Goal

The final project is no longer just the Required Function parking task.
The target system is an indoor smart delivery AMR with this integrated workflow:

`target selection -> straight-line corridor movement -> docking -> station verification -> cargo pickup or release -> telemetry upload -> dashboard display and task log`

The proposal originally focused on six modules:

1. motion control
2. docking and safety sensing
3. RFID station verification
4. cargo release
5. telemetry upload
6. dashboard and log visualization

Based on recent hardware changes, the practical implementation scope is now adjusted in two ways:

7. arm manipulation subsystem
   - a two-servo mechanical arm for taking or releasing objects
8. corridor-guidance subsystem
   - use two side-mounted ultrasonic sensors to observe left/right wall distance
   - use gyroscope feedback to keep heading stable while moving forward

## 2. Current Scope Decision

Current project direction:

- Required Function is treated as a completed baseline capability and reference.
- Proposal work is now the main implementation track.
- Arduino Mega remains the low-level real-time controller.
- Jetson Nano is the computer-side runtime for telemetry, dashboard, unlock page, and future higher-level logic.
- USB serial between Arduino and Jetson Nano is the main communication path for now.
- The robot will not use line-following color sensors for route control.
- Straight-line movement will be controlled by two side ultrasonics plus gyroscope, assuming the corridor walls are parallel and spacing is approximately constant.
- The new arm changes the original "servo door only" cargo-release assumption.

Things intentionally not fixed yet:

- whether RFID remains the final station verification method or becomes optional
- whether unlock by QR + fixed password remains a side demo or part of the final workflow
- whether the arm is used for pickup only, release only, or both
- how long-range movement should degrade safely if one wall reading becomes unstable

## 3. Current Hardware Baseline

Confirmed and important hardware now in scope:

- Arduino Mega main controller
- Jetson Nano as computer-side processor
- DC motor chassis
- dual side ultrasonic sensors
- MPU6050 gyroscope
- OLED
- new mechanical arm with two servos

### Newly added arm pins

- arm gripper open/close servo: `pin 44`
- arm up/down servo: `pin 45`

Reference sample code for a single servo exists at:

- [singleServo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/singleServo/singleServo.ino)

### Corridor-guidance assumption

For the current proposal implementation, straight-line movement is based on:

- left-side ultrasonic distance
- right-side ultrasonic distance
- gyroscope heading hold

Working assumption:

- the robot travels between approximately parallel walls
- wall spacing stays roughly constant over the section where straight-line control is needed
- if right wall distance increases relative to left, the robot has drifted left and needs right correction
- if left wall distance increases relative to right, the robot has drifted right and needs left correction

This replaces the previously discussed black/white line-following approach.

## 4. Repo Structure

Current relevant structure:

```text
.
|-- README.md
|-- plan.md
|-- agent.md
|-- firmware/
|   `-- arduino/
|       |-- proposal_amr/
|       |   `-- telemetry_serial_demo/
|       |       `-- telemetry_serial_demo.ino
|       |-- singleServo/
|       |   `-- singleServo.ino
|       |-- motortest/
|       |   `-- motortest.ino
|       `-- required_function_c1/
|           |-- PLAN.md
|           |-- final_code/
|           |-- final_code_optimized/
|           `-- multiple test sketches
|-- hardware/
|   `-- amr_pin_mapping_template.xlsx
|-- software/
|   |-- jetson_dashboard/
|   |   |-- server.py
|   |   |-- dashboard.html
|   |   |-- unlock.html
|   |   |-- unlock_config.json
|   |   `-- requirements.txt
|   `-- jetson_yolo_tests/
`-- Test_Code_24A/
```

Interpretation:

- `required_function_c1/` contains validated low-level references and completed parking-related logic.
- `proposal_amr/` should become the main Arduino-side proposal implementation area.
- `jetson_dashboard/` is already the seed of the computer-side proposal stack.

## 5. What Is Already Done

### Required Function baseline

The project already has a strong low-level base from Required Function work:

- motor control, PWM tuning, and directional validation
- dual ultrasonic usage
- gyro-based correction
- OLED output
- multiple sensor test sketches
- integrated parking state machines in:
  - [final_code.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/final_code/final_code.ino)
  - [final_code_optimized.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/final_code_optimized/final_code_optimized.ino)

This work should be reused as a reference base, not reimplemented from zero.

### Proposal communication baseline

Jetson-side telemetry and web stack already exist:

- [server.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/server.py)
- [dashboard.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/dashboard.html)
- [unlock.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock.html)

Current Jetson-side capabilities:

- read JSON telemetry from Arduino via USB serial
- show live dashboard in browser
- provide unlock page with fixed password
- send `OPEN_DOOR` command back to Arduino
- support mock mode when Arduino is not connected

### Proposal firmware draft

There is already a proposal Arduino-side telemetry draft:

- [telemetry_serial_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino)

Current role:

- provide a stable JSON telemetry schema
- provide a test path for Jetson-to-Arduino commands such as `OPEN_DOOR`

## 6. Updated Functional Breakdown

The real implementation is now best viewed as eight subsystems:

### A. Base locomotion

- motor drive
- straight-line tuning
- turning
- rotation

### B. Corridor guidance

- use left/right side ultrasonic readings to estimate corridor centering
- use gyroscope to stabilize heading and suppress drift
- assume both side walls are present and roughly parallel over the travel segment

### C. Docking and close-range safety

- use dual ultrasonic sensors for close approach and alignment
- reduce speed near station
- avoid collision

### D. Station verification

- read correct station identity
- initially still planned as RFID-based verification
- can be revised if actual demo setup changes

### E. Arm manipulation

- arm up/down servo on `45`
- arm gripper open/close servo on `44`
- support object taking and releasing

### F. Cargo handling / unlock interaction

- local or remote command can trigger open/close action
- QR unlock page currently targets this path
- final relationship between arm and unlock flow still needs to be fixed

### G. Telemetry and control bridge

- Arduino -> Jetson JSON telemetry
- Jetson -> Arduino command channel over same USB serial

### H. Dashboard and logs

- live status page
- unlock page
- task history and event record

## 7. Immediate Integration Risks

These are the main blockers right now:

1. Corridor-guidance control is not yet integrated
   - no main sketch yet combining left/right ultrasonic centering with gyro heading hold
   - correction thresholds and filtering are not tuned
   - fallback behavior for missing wall data is not fixed

2. Arm control is only at reference-sample level
   - no integrated proposal arm sketch yet
   - no pickup/release sequence yet

3. Proposal main state machine does not yet exist
   - telemetry and web bridge exist
   - real autonomous mission logic is not yet integrated into one main sketch

## 8. New Work Items

### Highest priority

1. Freeze the new hardware map.
   - update [amr_pin_mapping_template.xlsx](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/hardware/amr_pin_mapping_template.xlsx)
   - record the two arm servo pins
   - record the two side ultrasonic pins used for corridor guidance

2. Define the final proposal Arduino main sketch.
   - create one main entry point under `firmware/arduino/proposal_amr/`
   - reuse telemetry schema from `telemetry_serial_demo.ino`
   - reuse proven low-level motion logic from `required_function_c1`

3. Build arm control test sketches.
   - one sketch for gripper servo on `44`
   - one sketch for arm up/down servo on `45`
   - one combined arm test sketch

4. Build corridor-guidance test sketch.
   - read left/right side ultrasonic sensors
   - compute left-right difference and corridor center error
   - combine with gyro heading hold
   - verify correction direction on the chassis

5. Build gyro-assisted straight-line logic.
   - combine side-wall centering with heading hold
   - prevent oversteering and oscillation
   - define what to do when one wall temporarily disappears

### Medium priority

6. Merge corridor guidance and docking.
   - corridor guidance handles long-range route travel
   - ultrasonic handles final close approach near station

7. Integrate real arm action into task flow.
   - after station verification, decide whether to pick or release
   - define servo motion sequence and delays

8. Integrate real RFID verification.
   - target station ID input
   - read current station
   - mismatch handling

9. Upgrade Jetson dashboard from draft to mission UI.
   - include new arm state
   - include unlock state
   - include corridor-guidance state
   - add clearer logs and fault display

### Lower priority / later decision

10. Decide the final role of QR unlock.
   - side demo only
   - operator override
   - official delivery interaction

11. Decide whether YOLO stays separate or joins the mission.
   - currently it is only a prepared side capability

## 9. Proposed Final State Machine Direction

A practical high-level mission flow now looks like:

`IDLE -> START -> CORRIDOR_DRIVE -> APPROACH_STATION -> DOCK_AND_ALIGN -> VERIFY_STATION -> ARM_ACTION -> REPORT_STATUS -> NEXT_TASK or DONE`

Possible sub-behaviors:

- `CORRIDOR_DRIVE`
  - use 2 side ultrasonics + gyro
- `DOCK_AND_ALIGN`
  - use ultrasonic + gyro
- `VERIFY_STATION`
  - use RFID
- `ARM_ACTION`
  - move arm down/up and open/close gripper
- `REPORT_STATUS`
  - send telemetry to Jetson
  - update dashboard and log

## 10. Module Status Snapshot

| Module | Status | Notes |
|---|---|---|
| Base motion control | Strong reference baseline ready | Derived from Required Function and existing motor tests |
| Corridor straight-line control with side ultrasonics + gyro | Planned, not implemented | Replaces the cancelled 3-sensor line-following approach |
| Docking and safety sensing | Reference baseline ready | Ultrasonic and gyro logic already exist from Required Function |
| RFID verification | Not integrated | Still planned, not yet wired into proposal flow |
| Arm manipulation | Newly added, not integrated | Two servo pins fixed at `44` and `45`, only sample code exists |
| Cargo handling / unlock | Partial | Jetson fixed-password unlock exists, real arm-based handling not integrated |
| Telemetry upload | Working draft | Serial JSON and Jetson bridge already exist |
| Dashboard and logs | Working draft | Live dashboard and unlock page exist, mission UI still incomplete |

## 11. Immediate Next Step Order

1. Update hardware pin mapping with the arm and the two side ultrasonic sensors used for corridor guidance.
2. Write arm servo test sketches based on [singleServo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/singleServo/singleServo.ino).
3. Write a two-side-ultrasonic corridor test sketch with OLED/serial feedback.
4. Build a first straight-line corridor control sketch using side ultrasonics + gyro.
5. Define the final proposal Arduino state names and telemetry fields.
6. Create the first integrated proposal mission sketch under `proposal_amr`.
7. Merge docking, RFID, arm action, telemetry, and dashboard into one end-to-end demo flow.

## 12. Notes For Future Threads

- Treat this file as the current project-wide planning document.
- `required_function_c1/PLAN.md` remains a historical plan for the completed Required Function phase.
- Do not assume color-sensor-based route following is still part of scope; it has been dropped.
- Do not mark the arm subsystem or corridor-guidance subsystem as complete until they are integrated into the proposal workflow, not only tested in isolation.

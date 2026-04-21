# ELEC3848 Project Plan

Last updated: 2026-04-21

This file is the current project-level source of truth.
It merges:

- the original proposal workflow
- the completed Required Function baseline
- the current Jetson Nano dashboard / telemetry work
- the current YOLO test status
- the mechanical arm update
- the latest hardware cleanup: side ultrasonics removed

## 1. Project Goal

The project is no longer just the Required Function parking task.
The target system is an indoor smart delivery AMR with this integrated workflow:

`target selection -> gyro-assisted straight travel -> docking -> station verification -> cargo pickup or release -> telemetry upload -> dashboard display and task log`

The practical subsystem list is now:

1. base locomotion
2. docking and close-range safety
3. station verification
4. arm manipulation
5. cargo handling / unlock interaction
6. telemetry upload
7. dashboard and logs
8. optional vision / obstacle experiments on Jetson Nano

## 2. Current Scope Decision

Current project direction:

- Required Function is treated as completed baseline capability and reference.
- Proposal work is the active implementation track.
- Arduino Mega remains the low-level real-time controller.
- Jetson Nano remains the computer-side runtime for telemetry, dashboard, unlock page, and future high-level logic.
- USB serial between Arduino and Jetson Nano is the main communication path for now.
- Line-following color sensors remain cancelled.
- The two side-mounted ultrasonic sensors are removed from the hardware plan.
- Long-range straight movement must no longer depend on corridor wall-distance balancing.
- Straight-line control should be based on:
  - MPU6050 heading hold
  - per-wheel PWM tuning / trim
  - calibration results stored for different motion modes and speed tiers
- The front ultrasonic pair remains valid and should still be used for docking, close-range stopping, parking, and safety.

Things intentionally not fixed yet:

- whether RFID remains the final station verification method or becomes optional
- whether unlock by QR + fixed password remains a side demo or part of the final workflow
- whether the arm is used for pickup only, release only, or both
- whether YOLO stays separate or joins the final demo path

## 3. Current Hardware Baseline

Confirmed and important hardware now in scope:

- Arduino Mega main controller
- Jetson Nano as computer-side processor
- DC motor chassis
- front ultrasonic pair
- MPU6050 gyroscope
- OLED
- mechanical arm with two servos

Removed from current hardware baseline:

- left side ultrasonic
- right side ultrasonic

### Arm pins

- gripper open/close servo: `pin 44`
- arm up/down servo: `pin 45`

### Navigation / motion assumption

For the current proposal implementation:

- forward / backward straightness should come from gyro heading hold plus wheel PWM trim
- strafe should also use gyro to suppress yaw, but strafe calibration remains only semi-automatic without encoder or lateral position sensing
- docking / parking can still use the front ultrasonic pair

This replaces the previously discussed corridor-guidance concept based on side wall distance.

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
|       |   |-- telemetry_serial_demo/
|       |   |-- motor_calibration_assistant/
|       |   `-- obstacle_avoidance_test/        (historical / obsolete)
|       |-- singleServo/
|       `-- required_function_c1/
|           |-- final_code/
|           |-- final_code_optimized/           (historical / obsolete)
|           |-- motion_ultrasonic_parking_test/
|           |-- straight_line_power_test/
|           `-- dual_ultrasonic_oled_test/      (historical / obsolete)
|-- hardware/
|   `-- amr_pin_mapping_template.xlsx
|-- software/
|   |-- jetson_dashboard/
|   `-- jetson_yolo_tests/
`-- Test_Code_24A/
```

Interpretation:

- `required_function_c1/` contains validated low-level references and parking-related logic.
- `proposal_amr/` is the active Arduino-side proposal area.
- `telemetry_serial_demo/` is the active Jetson bridge draft and should follow the latest motion decision.
- `motor_calibration_assistant/` is the active calibration utility for gyro + per-wheel PWM tuning.
- the files explicitly marked `historical / obsolete` should not be treated as current hardware targets.

## 5. What Is Already Done

### Required Function baseline

The project already has a strong low-level base from Required Function work:

- motor control, PWM tuning, and directional validation
- front ultrasonic usage
- gyro-based correction
- OLED output
- multiple sensor test sketches
- integrated parking state machines in:
  - [final_code.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/final_code/final_code.ino)
  - [motion_ultrasonic_parking_test.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/motion_ultrasonic_parking_test/motion_ultrasonic_parking_test.ino)

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

### Current motion / calibration drafts

There are now two active Arduino-side motion support sketches:

- [telemetry_serial_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino)
  - proposal-stage telemetry bridge
  - manual control with gyro heading hold + per-wheel trim
  - keeps the front ultrasonic telemetry path

- [motor_calibration_assistant.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/motor_calibration_assistant/motor_calibration_assistant.ino)
  - stores per-mode / per-tier / per-motor PWM profiles
  - supports EEPROM save / load
  - supports gyro-based auto search for forward / backward
  - supports semi-auto heading-only search for strafe

## 6. Updated Functional Breakdown

### A. Base locomotion

- motor drive
- per-wheel PWM trim
- gyro heading hold
- forward / backward / strafe test and calibration

### B. Docking and close-range safety

- use the front ultrasonic pair for approach, stopping, and short-range alignment
- reduce speed near station
- avoid collision

### C. Station verification

- read correct station identity
- originally planned as RFID-based verification
- still subject to demo simplification if needed

### D. Arm manipulation

- arm up/down servo on `45`
- arm gripper open/close servo on `44`
- support object taking and releasing

### E. Cargo handling / unlock interaction

- local or remote command can trigger open/close action
- QR unlock page currently targets this path
- final relationship between arm and unlock flow still needs to be fixed

### F. Telemetry and control bridge

- Arduino -> Jetson JSON telemetry
- Jetson -> Arduino command channel over the same USB serial link

### G. Dashboard and logs

- live status page
- unlock page
- task history and event record

### H. YOLO experiment path

- separate Jetson-side obstacle / vision experiments
- not yet part of the guaranteed mission path

## 7. Immediate Integration Risks

These are the main blockers right now:

1. Final proposal Arduino mission state machine still does not exist.
   - locomotion, docking, arm, verification, and reporting are not yet integrated into one main sketch

2. No encoder is installed.
   - forward / backward tuning can only be “gyro-straighter,” not true speed calibration
   - strafe results remain initial values, not rigorous calibration

3. Arm control is not yet integrated into a full task sequence.
   - servo pins are fixed
   - pickup / release sequencing is still separate work

4. RFID verification is not yet integrated into the proposal workflow.

5. YOLO runtime is still split across two environments.
   - Docker runtime works on Jetson Nano
   - host-side `venv` setup is still incomplete

## 8. New Work Items

### Highest priority

1. Freeze the updated hardware map.
   - keep the two arm servo pins
   - keep the front ultrasonic pair
   - remove side-ultrasonic assumptions from active planning and active code

2. Define the final proposal Arduino main sketch.
   - create one main entry point under `firmware/arduino/proposal_amr/`
   - reuse telemetry schema from `telemetry_serial_demo.ino`
   - reuse proven low-level motion logic from `required_function_c1`

3. Integrate gyro-based straight motion.
   - apply calibrated per-wheel PWM profiles
   - keep heading with MPU6050
   - avoid reintroducing side wall-distance dependencies

4. Integrate docking with front ultrasonic pair.
   - long-range travel uses gyro + PWM trim
   - final approach still uses front ultrasonic sensing

5. Integrate arm control into the proposal path.
   - pickup / release sequence
   - state feedback into telemetry and dashboard

### Medium priority

6. Integrate real RFID verification.

7. Upgrade Jetson dashboard from draft to mission UI.
   - include arm state
   - include unlock state
   - include motion / calibration related status when useful

8. Evaluate YOLO-based obstacle detection integration.
   - keep it as a separate test path first
   - connect Jetson detection results with Arduino behavior over USB serial
   - decide later whether it belongs in the final demo flow

### Lower priority / later decision

9. Decide the final role of QR unlock.

10. Decide whether YOLO stays experimental or becomes a mission subsystem.

## 9. Proposed Final State Machine Direction

A practical high-level mission flow now looks like:

`IDLE -> START -> GYRO_STRAIGHT_DRIVE -> APPROACH_STATION -> DOCK_AND_ALIGN -> VERIFY_STATION -> ARM_ACTION -> REPORT_STATUS -> NEXT_TASK or DONE`

Possible sub-behaviors:

- `GYRO_STRAIGHT_DRIVE`
  - use per-wheel PWM trim + gyro heading hold
- `DOCK_AND_ALIGN`
  - use front ultrasonic pair + gyro
- `VERIFY_STATION`
  - use RFID or an accepted simplified substitute
- `ARM_ACTION`
  - move arm down/up and open/close gripper
- `REPORT_STATUS`
  - send telemetry to Jetson
  - update dashboard and log

## 10. Task Types And Full-Mission Definition

The UI should eventually support explicit task selection.
Current planned task set:

1. Full mission
   - complete pickup + cargo loading + delivery + user unlock flow
2. Obstacle-avoidance mission
   - isolated test / demo path for obstacle handling behavior
3. Pickup mission
   - drive to pickup point and complete the grab / load sequence only
4. Delivery mission
   - drive to delivery point and complete the handoff / unlock sequence only
5. Manual test
   - no autonomous mission logic
   - only UI-based manual control of chassis and arm

### Full-mission flow

The current intended full mission is:

`START -> STRAIGHT_TRAVEL_TO_PICKUP -> RFID_PICKUP_DETECTED -> PICKUP_SEQUENCE -> OPEN_BOX -> LOAD_ITEM -> CLOSE_BOX -> TURN_AROUND -> STRAIGHT_TRAVEL_TO_DELIVERY -> RFID_DELIVERY_DETECTED -> WAIT_USER_UNLOCK -> USER_OPEN_DOOR -> USER_CLOSE_DOOR or AUTO_CLOSE_AFTER_60S -> REPORT_STATUS -> DONE`

Detailed interpretation:

- the robot first drives straight until the RFID reader detects the pickup card / pickup station
- once pickup RFID is detected, the robot enters the pickup sequence
- the arm manipulates the item
- the box door opens
- the item is placed into the box
- the box door closes
- the robot turns around
- the robot then drives toward the delivery point
- when the RFID reader detects the delivery card / delivery station, the robot stops
- the system waits for the user to scan the QR code and enter the password to open the door
- after the user opens the door and takes the item:
  - the user may manually close the door
  - if the user does not close it manually, the system starts a 60-second countdown and then closes the door automatically

### Obstacle-avoidance interruption rule

Obstacle avoidance is not treated as a separate timeline isolated from all other work.
Instead:

- obstacle handling may interrupt almost any active mission stage
- if YOLO detects an obstacle, the system may temporarily enter the obstacle-avoidance path
- after obstacle handling ends, the system should return to the interrupted mission stage or to the nearest safe recovery point
- this interruption model must be reflected consistently in:
  - UI task display
  - Jetson task state
  - Arduino execution state
  - any future interrupt / resume mechanism

## 11. Module Status Snapshot

| Module | Status | Notes |
|---|---|---|
| Base motion control | Strong reference baseline ready | Derived from Required Function and existing motor tests |
| Gyro straight-line control with per-wheel PWM trim | Active draft available | Side-ultrasonic corridor guidance has been removed |
| Docking and safety sensing | Reference baseline ready | Front ultrasonic and gyro logic already exist from Required Function |
| RFID verification | Not integrated | Still planned, not yet wired into proposal flow |
| Arm manipulation | Partially prepared | Servo pins fixed at `44` and `45`, still needs workflow integration |
| Cargo handling / unlock | Partial | Jetson fixed-password unlock exists, full arm-based handling not integrated |
| Telemetry upload | Working draft | Serial JSON and Jetson bridge already exist |
| Dashboard and logs | Working draft | Live dashboard and unlock page exist, mission UI still incomplete |
| YOLO vision on Jetson Nano | Experimental runtime ready | Docker runtime works, host venv incomplete, may interrupt missions for obstacle handling |

## 12. Immediate Next Step Order

1. Keep the hardware / code / docs aligned with the “no side ultrasonic” decision.
2. Continue tuning and validating gyro + per-wheel PWM profiles.
3. Define the final proposal task names, mission stages, and telemetry fields so UI, Jetson, and Arduino use the same task vocabulary.
4. Add the task-selection UI layer and reserve interfaces for later runtime binding.
5. Create the first integrated proposal mission sketch under `proposal_amr`.
6. Merge docking, verification, arm action, telemetry, dashboard, and any accepted YOLO behavior into one end-to-end demo flow.

## 13. Notes For Future Threads

- Treat this file as the current project-wide planning document.
- `required_function_c1/PLAN.md` remains a historical plan for the completed Required Function phase.
- Do not assume color-sensor-based route following is still part of scope; it has been dropped.
- Do not assume side ultrasonics are still installed; they were removed from the current hardware baseline on 2026-04-21.
- Keep the distinction clear:
  - front ultrasonic pair: still valid and active
  - side ultrasonics: removed and obsolete
- The current planned UI task list is:
  - Full mission
  - Obstacle-avoidance mission
  - Pickup mission
  - Delivery mission
  - Manual test
- Do not mark the arm subsystem or proposal locomotion subsystem as complete until they are integrated into the proposal workflow, not only tested in isolation.

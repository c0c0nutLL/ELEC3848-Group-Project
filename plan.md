# ELEC3848 Proposal Implementation Plan

Last updated: 2026-04-15

This file is the shared progress tracker for the proposal-based implementation.
Other threads should use this file as the main sync point and update it when proposal-related progress changes.

## 1. Proposal Functionality

The proposal defines one integrated indoor smart delivery AMR workflow:

`target selection -> robot movement -> docking -> RFID verification -> cargo release -> telemetry upload -> dashboard display and task log`

The proposal functionality is split into six modules:

1. Motion control
   - Drive the AMR to the target station.
   - Support forward motion, turning, and basic trajectory execution.
2. Docking and safety sensing
   - Use ultrasonic sensing to approach the target station safely.
   - Slow down, align, and stop before delivery.
3. RFID station verification
   - Read the station RFID tag.
   - Compare detected station ID with the assigned target.
   - Reject delivery if the station does not match.
4. Cargo release
   - Use a servo-controlled door or release mechanism.
   - Only open after successful verification.
5. Telemetry upload
   - Send robot state, docking data, RFID result, and delivery result to Jetson Nano or PC.
6. Dashboard and log visualization
   - Show robot status in real time.
   - Record task history for later debugging, screenshots, and demonstration.

Current scope decision:

- No map / GPS / SLAM feature is included for now.
- Jetson Nano is the target Python runtime for computer-side code.

## 2. Overall Project Structure

Current proposal-related structure in the repo:

```text
.
|-- README.md
|-- plan.md
|-- firmware/
|   `-- arduino/
|       |-- proposal_amr/
|       |   `-- telemetry_serial_demo/
|       |       `-- telemetry_serial_demo.ino
|       `-- motortest/
|           `-- motortest.ino
|-- hardware/
|   `-- amr_pin_mapping_template.xlsx
|-- software/
|   |-- jetson_dashboard/
|   |   |-- server.py
|   |   |-- dashboard.html
|   |   |-- unlock.html
|   |   |-- unlock_config.json
|   |   |-- requirements.txt
|   |   |-- JETSON_NANO_ENVIRONMENT.md
|   |   `-- YOLO_DEPLOYMENT_JETSON_NANO.md
|   `-- jetson_yolo_tests/
|       |-- image_detection_test.py
|       |-- camera_detection_test.py
|       `-- README.md
`-- Test_Code_24A/
    `-- Test Code/
        |-- Car_Volt_Feedback_24A/
        |   `-- Car_Volt_Feedback_24A.ino
        `-- car.apk
```

Directories not treated as the proposal implementation baseline:

- `Car_Libs/`
- `reflection/`
- `required_function`-type material
- `firmware/arduino/required_function_c1/`

Those directories may contain references, but proposal progress should not be tracked against them.

## 3. Already Completed

### Project Definition

- The proposal scope is clear and stable:
  - indoor smart delivery AMR
  - six-module workflow
  - Jetson Nano used on the computer side
- The repository now has a proposal-oriented software layout under `firmware/` and `software/`.

### Firmware Side

- A proposal demo sketch exists at:
  - [telemetry_serial_demo.ino](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\firmware\arduino\proposal_amr\telemetry_serial_demo\telemetry_serial_demo.ino)
- This demo already provides:
  - periodic JSON telemetry over serial using the Jetson dashboard field names
  - real serial motor commands for `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, and `STOP`
  - live ultrasonic reporting for `right_cm`, `left_cm`, `avg_cm`, and `diff_cm`
  - a serial command path for `OPEN_DOOR`
  - a temporary remote-unlock state for Jetson-to-Arduino integration testing

### Jetson Dashboard Side

- A Jetson-side serial bridge and dashboard draft already exists under:
  - [server.py](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\software\jetson_dashboard\server.py)
  - [dashboard.html](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\software\jetson_dashboard\dashboard.html)
  - [unlock.html](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\software\jetson_dashboard\unlock.html)
- The current Jetson dashboard stack already supports:
  - reading JSON telemetry from Arduino through serial
  - a dashboard page
  - manual motor control from the dashboard page through Jetson-to-Arduino serial commands
  - motor power control from a browser slider
  - an unlock page
  - fixed-password verification
  - sending `OPEN_DOOR` back to Arduino
  - mock mode for browser-side testing without Arduino
  - merged control state and telemetry updates through the same SSE stream

### Jetson Environment And YOLO Preparation

- Jetson environment notes have been documented in:
  - [JETSON_NANO_ENVIRONMENT.md](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\software\jetson_dashboard\JETSON_NANO_ENVIRONMENT.md)
- YOLO deployment notes for Jetson Nano have been documented in:
  - [YOLO_DEPLOYMENT_JETSON_NANO.md](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\software\jetson_dashboard\YOLO_DEPLOYMENT_JETSON_NANO.md)
- Basic YOLO test scripts have been prepared in:
  - [image_detection_test.py](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\software\jetson_yolo_tests\image_detection_test.py)
  - [camera_detection_test.py](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\software\jetson_yolo_tests\camera_detection_test.py)

### Existing Reference Assets

- There is older Arduino test code that can be reused selectively for low-level hardware behavior:
  - [Car_Volt_Feedback_24A.ino](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\Test_Code_24A\Test Code\Car_Volt_Feedback_24A\Car_Volt_Feedback_24A.ino)
- This file is a reference source for motor control, servo control, OLED usage, Bluetooth control, and battery feedback, but it is not yet the proposal main firmware.

## 4. Next Work Items

### Highest Priority

1. Build the real proposal firmware state machine.
   - Replace manual-control demo state reporting with real hardware-driven states.
   - Define one main Arduino entry point for the proposal workflow.
2. Finalize hardware pin mapping.
   - Confirm motor, ultrasonic, RFID, servo, and serial wiring.
   - Fill the hardware mapping document with actual project pins.
3. Connect the cargo release path to a real servo action.
   - Replace the current demo-only `OPEN_DOOR` behavior with actual servo control.
4. Integrate real docking sensing.
   - Read ultrasonic distance values.
   - implement slow-down and stop thresholds.
5. Integrate RFID verification.
   - Read tag IDs.
   - compare against target station.
   - block delivery on mismatch.

### Medium Priority

6. Promote the Jetson dashboard from draft to proposal status UI.
   - Align displayed fields with the final autonomous Arduino telemetry schema.
   - make task history and failure states clearer.
7. Define the final telemetry protocol.
   - Freeze field names and state names shared by Arduino and Jetson.
   - keep backward compatibility where possible.
8. Prepare Jetson-side YOLO runtime when Docker is ready.
   - validate image detection
   - validate `/dev/video0` live detection
   - decide whether YOLO output will feed the proposal workflow or stay as an isolated capability

### Lower Priority / Later Integration

9. Decide whether QR unlock remains in the final demo.
   - If yes, constrain it by robot state.
   - If no, keep it as a side demo only.
10. Add test records and demonstration scripts.
   - capture screenshots
   - record expected state transitions
   - log failure cases and recovery behavior

## Module Status Snapshot

| Module | Status | Notes |
|---|---|---|
| Motion control | Partial manual-control path ready | Web -> Jetson -> Arduino manual drive now works, but the proposal autonomous state machine is not integrated yet |
| Docking and safety sensing | Not started in proposal firmware | Still needs real ultrasonic integration |
| RFID verification | Not started in proposal firmware | No integrated proposal sketch yet |
| Cargo release | Partial demo only | `OPEN_DOOR` path exists, real servo delivery logic not finished |
| Telemetry upload | Usable for integration | Serial JSON telemetry now reports live motion state and ultrasonic data while remaining Jetson-compatible |
| Dashboard and logs | Usable for integration | Dashboard now includes live telemetry, manual drive controls, and reduced log spam, but still needs final proposal polish |

## Notes For Other Threads

- Treat this file as the proposal implementation source of truth.
- Do not mark a module as done unless it is integrated into the proposal workflow, not only tested in isolation.
- Reference-only code is useful, but it does not count as completed proposal functionality.
- If a thread changes the Arduino telemetry schema, this file and the Jetson dashboard documentation should be updated together.

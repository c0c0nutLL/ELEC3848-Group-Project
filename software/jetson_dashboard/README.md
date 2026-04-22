# Jetson Dashboard

This directory contains the current Jetson Nano web UI, serial bridge, and task-selection placeholder APIs.

Current data path:

- `Arduino -> USB serial -> Jetson Nano -> browser`
- `Browser -> Jetson HTTP API -> USB serial -> Arduino`
- `Phone -> /unlock -> password verify -> OPEN_DOOR / CLOSE_DOOR -> Arduino`

## Main Files

- [server.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/server.py)
  - main Jetson service
  - reads Arduino JSON telemetry over serial
  - serves the home page, split UI pages, dashboard, unlock page, and HTTP APIs
  - handles interrupt/resume demo control
  - exposes task-selection placeholder APIs
- [home.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/home.html)
  - unified UI entry page
  - links to all major pages
  - shows a lightweight live system snapshot
- [dashboard.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/dashboard.html)
  - original combined dashboard
  - still contains telemetry, task selection, manual controls, arm controls, and debug blocks
  - kept as a compatibility page
- [manual.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/manual.html)
  - dedicated manual-control page
  - includes both chassis control and arm control
- [tasks.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/tasks.html)
  - dedicated task-selection page
  - includes placeholder task presets and interrupt/resume demo controls
- [status.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/status.html)
  - dedicated live system-status page
  - shows bridge, mode, sensors, actuators, and task placeholder snapshot
- [debug.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/debug.html)
  - dedicated debug page
  - shows event log, raw telemetry JSON, and raw task-selection JSON
- [unlock.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock.html)
  - unlock page for phone access
- [unlock_config.json](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock_config.json)
  - fixed password, open/close command, and auto-close config

## Page Structure

The UI is now organized around a clear home-page entry model:

- `/`
  - `Home`
  - default entry page for the whole UI system
- `/dashboard`
  - original combined dashboard
  - kept for compatibility and full-surface access
- `/telemetry`
  - telemetry page entry
  - currently serves the same page as `/dashboard`
- `/manual`
  - dedicated manual + arm page
- `/arm`
  - redirects to `/manual`
- `/tasks`
  - dedicated task-selection page
- `/status`
  - dedicated system-status page
- `/debug`
  - dedicated debug page
- `/unlock`
  - dedicated unlock page

Why this structure was chosen:

- it keeps the current dashboard working
- it gives users a real `Home` page
- it removes the old requirement to jump to anchors inside the dashboard for manual, task, status, and debug work
- it keeps route names stable for future expansion

## What Is Real Vs Placeholder

Real standalone pages today:

- `/`
- `/dashboard`
- `/telemetry`
- `/manual`
- `/tasks`
- `/status`
- `/debug`
- `/unlock`

Real implemented controls today:

- manual chassis control
- PWM adjustment
- arm lift / gripper control
- unlock page
- live telemetry display
- raw debug display
- interrupt/resume demo controls

Still placeholder / reserved:

- task selection is still a UI/API placeholder
- task-selection routes do not start the final proposal mission runtime yet
- the dedicated task page is structurally real, but the mission state behind it is still mock / placeholder

## Paired Arduino Sketches

Legacy manual-only sketch:

- [telemetry_serial_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino)

Interrupt / resume demo sketch:

- [interrupt_resume_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino)

Important behavior:

- the UI still works with the legacy manual sketch
- interrupt / resume buttons stay unavailable until Arduino telemetry includes `mode_protocol_version`
- the task-selection module is separate from the current interrupt/resume demo and does not start the real task runtime yet

## Runtime Model

The current interrupt / resume demo expects two layers of state when the paired Arduino sketch is running:

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

Important rule:

- UI `Take Manual Control` changes the run mode
- it does not erase the auto stage
- the saved resume stage is tracked separately in `saved_auto_stage`

## Home Page

The `Home` page is the unified UI entry.

What it does:

- presents the whole UI as one navigation system
- shows current bridge, robot, mode, auto-stage, and task-selection snapshot
- links to all major feature pages
- avoids forcing users to edit URLs manually

What it does not do:

- it does not replace existing page logic
- it does not add a frontend framework
- it does not change the current control APIs

## Split Pages

### Telemetry

- route: `/telemetry`
- current implementation: serves the existing combined dashboard page
- purpose: preserve the live telemetry view

### Manual + Arm

- route: `/manual`
- arm alias route: `/arm`
- purpose: dedicated page for:
  - manual override buttons
  - drive pad
  - PWM control
  - arm lift and gripper control

### Task Selection

- route: `/tasks`
- purpose: dedicated page for:
  - placeholder task presets
  - placeholder task state
  - interrupt/resume demo controls

### System Status

- route: `/status`
- purpose: dedicated read-only status page for:
  - run mode
  - bridge health
  - sensors
  - actuators
  - task placeholder snapshot

### Debug

- route: `/debug`
- purpose: dedicated debug page for:
  - recent event log
  - raw telemetry JSON
  - raw task-selection JSON

## Original Combined Dashboard

The original `/dashboard` page is still available.

It now acts as a compatibility page:

- it still exposes the full combined UI
- it still contains task selection, manual controls, arm controls, and debug blocks
- new navigation now prefers the split pages instead of routing users into dashboard anchors

## Task Selection Placeholder

The task-selection model is intentionally separate from the current interrupt/resume demo.

It only reserves UI and HTTP contracts for the future proposal task runtime.

What it does now:

- lets the user choose one of five mission presets
- shows task description and placeholder task state
- exposes `start / stop / resume` buttons for UI validation
- keeps a mock `savedStage` / `resumeAvailable` flow so the resume UI can be tested before the backend exists

Current placeholder alignment decisions:

- `FULL_MISSION`, `PICKUP_MISSION`, and `DELIVERY_MISSION` are the frozen task names for the proposal path
- placeholder entry stages now mirror the planned mission vocabulary:
  - `TRAVEL_TO_PICKUP`
  - `TRAVEL_TO_DELIVERY`
  - `OBSTACLE_MONITOR`
- RFID is the primary station-verification method
- QR + fixed password unlock is part of the final delivery flow

What it does not do yet:

- it does not start the current Arduino auto demo
- it does not send final proposal task commands
- it does not own the real Jetson or Arduino mission state machine

Current placeholder state model:

```json
{
  "selectedTask": "FULL_MISSION",
  "taskStatus": "IDLE",
  "currentStage": "NONE",
  "resumeAvailable": false,
  "savedStage": null,
  "isManualTest": false,
  "isObstacleInterruptEnabled": true,
  "obstacleInterruptActive": false
}
```

## HTTP APIs

Existing APIs:

- `POST /api/control`
- `POST /api/arm`
- `POST /api/unlock`

`POST /api/unlock` accepts either `{"action": "OPEN", "password": "3848"}` or `{"action": "CLOSE"}`.

Interrupt / resume demo APIs:

- `POST /api/task`
- `POST /api/mode`

Task-selection placeholder APIs:

- `GET /api/task-selection/state`
- `POST /api/task-selection/select`
- `POST /api/task-selection/start`
- `POST /api/task-selection/stop`
- `POST /api/task-selection/resume`
- `POST /api/task-selection/mock-update`

Relevant page URLs also appear in `/api/config` under `http_paths`.

Examples:

`POST /api/task`

```json
{
  "action": "START",
  "source": "dashboard"
}
```

`POST /api/mode`

```json
{
  "action": "TAKE_MANUAL",
  "source": "dashboard"
}
```

`POST /api/task-selection/select`

```json
{
  "selectedTask": "FULL_MISSION",
  "source": "tasks-page"
}
```

## Serial Protocol

Jetson sends these task-level commands to the current interrupt/resume demo sketch:

- `TASK START`
- `TASK ABORT`
- `OVERRIDE TAKE`
- `OVERRIDE RESUME`
- `OVERRIDE ABORT`

The existing realtime commands remain:

- `DRIVE FORWARD|BACKWARD|LEFT|RIGHT|STOP <PWM>`
- `ARM LIFT|GRIP <ANGLE>`
- `OPEN_DOOR`
- `GYRO`

Telemetry fields added by the interrupt/resume demo sketch:

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

Task-selection placeholder APIs do not send serial commands yet.

## Resume Rules

The current interrupt/resume implementation is honest about recovery limits.

What is supported:

- stage-level resume
- freezing the auto stage machine while manual override is active
- rejecting manual drive / arm commands outside manual override
- rejecting ordinary resume when a fault is latched

What is not claimed:

- instruction-level resume
- sub-millisecond actuator recovery
- position-perfect resume after the robot is manually displaced

Stage behavior in the paired demo sketch:

- `CORRIDOR_DRIVE`, `APPROACH_STATION`, `DOCK_AND_ALIGN`, `VERIFY_STATION`, and `REPORT_STATUS`
  - resume by re-entering the same stage
- `ARM_ACTION`
  - resume from the safe entry of the arm sequence, not from the middle of a servo move

Task-selection placeholder behavior:

- `resumeAvailable` is mock state only
- `savedStage` is mock state only
- no real task execution is resumed until a backend task manager is wired in

## Serial And Startup

On the Jetson Nano side, the default serial device is:

- `/dev/ttyUSB0`

Start the service with:

```bash
cd software/jetson_dashboard
pip3 install -r requirements.txt
python3 server.py --serial /dev/ttyUSB0 --baud 115200 --http-port 8080
```

If the board enumerates differently, override the path explicitly, for example:

- `/dev/ttyUSB0`
- `/dev/ttyACM0`
- `/dev/ttyACM1`

## Default Safety Behavior

The current defaults remain conservative:

- `server.py` binds to `127.0.0.1` by default
- `/api/control`, `/api/arm`, `/api/task`, and `/api/mode` accept loopback clients only
- task-selection `POST` routes also accept loopback clients only
- realtime commands are rejected while the serial bridge is disconnected
- queued commands are dropped when the serial bridge disconnects

Local access:

- Home: `http://127.0.0.1:8080/`
- Dashboard: `http://127.0.0.1:8080/dashboard`
- Telemetry: `http://127.0.0.1:8080/telemetry`
- Manual + Arm: `http://127.0.0.1:8080/manual`
- Tasks: `http://127.0.0.1:8080/tasks`
- Status: `http://127.0.0.1:8080/status`
- Debug: `http://127.0.0.1:8080/debug`
- Unlock: `http://127.0.0.1:8080/unlock`

If phone access to `/unlock` is needed, expose the service explicitly:

```bash
python3 server.py --serial /dev/ttyUSB0 --baud 115200 --http-port 8080 --host 0.0.0.0
```

Notes:

- `/unlock` can then be reached from another device on the LAN
- `/api/control`, `/api/arm`, `/api/task`, and `/api/mode` remain loopback-only

## Mock Mode

Use mock mode to test the browser and HTTP APIs without Arduino:

```bash
python3 server.py --mock --http-port 8080
```

In mock mode:

- the UI still receives live updates
- the home page still shows a live snapshot
- the task-selection placeholder module still keeps its state in memory
- interrupt / resume controls are visible and enabled
- manual control gating behaves like the real interrupt/resume protocol
- `OPEN_DOOR` opens the simulated cargo door and starts the configured auto-close timer
- `CLOSE_DOOR` closes the simulated cargo door immediately

Mock mode is still only for UI / API validation.

It is not a motor-control simulation.

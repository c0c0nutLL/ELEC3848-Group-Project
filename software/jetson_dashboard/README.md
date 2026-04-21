# Jetson Dashboard

This directory contains the current Jetson Nano web dashboard, home page, unlock page, and serial bridge.

Current path:

- `Arduino -> USB serial -> Jetson Nano -> browser`
- `Browser -> Jetson HTTP API -> USB serial -> Arduino`
- `Phone -> /unlock -> password verify -> OPEN_DOOR -> Arduino`

## Main Files

- [server.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/server.py)
  - main Jetson service
  - reads Arduino JSON telemetry over serial
  - serves the home page, dashboard, unlock page, and HTTP APIs
  - handles interrupt/resume demo control
  - exposes task-selection placeholder APIs
- [home.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/home.html)
  - unified UI entry page
  - links to all major pages and dashboard sections
  - shows a lightweight live system snapshot
- [dashboard.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/dashboard.html)
  - existing combined telemetry/control page
  - task-selection placeholder UI
  - interrupt / resume demo controls
  - manual drive controls
  - PWM slider
  - arm controls
  - debug log and raw telemetry
- [unlock.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock.html)
  - unlock page for phone access
- [unlock_config.json](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock_config.json)
  - fixed password and unlock command config

## Page Structure

The UI is now organized around a simple and explicit entry model:

- `/`
  - `Home`
  - default entry page for the whole UI system
- `/dashboard`
  - existing combined dashboard page
- `/unlock`
  - existing unlock page

Additional stable entry URLs are now provided for major feature areas:

- `/telemetry`
  - redirects to the telemetry/status section inside `/dashboard`
- `/manual`
  - redirects to the manual drive section inside `/dashboard`
- `/tasks`
  - redirects to the task-selection section inside `/dashboard`
- `/arm`
  - redirects to the arm-control section inside `/dashboard`
- `/status`
  - redirects to the live status area inside `/dashboard`
- `/debug`
  - redirects to the event log / raw telemetry area inside `/dashboard`

Why this structure was chosen:

- it keeps the current dashboard intact
- it gives users a clear `Home` page instead of forcing URL guessing
- it exposes stable URLs now, even before the UI is split into more standalone pages later

## What Is Real Vs Placeholder

Real standalone pages today:

- `/`
- `/dashboard`
- `/unlock`

Real existing feature areas inside `/dashboard`:

- telemetry
- task selection placeholder
- interrupt / resume demo controls
- manual control
- arm control
- event log
- raw telemetry / debug

Reserved or placeholder behavior:

- task selection is still a UI/API placeholder
- the task-selection routes do not start a real proposal mission state machine yet
- the new section-entry URLs such as `/manual` or `/tasks` are redirects into the current dashboard, not separate pages yet

## Paired Arduino Sketches

Legacy manual-only sketch:

- [telemetry_serial_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino)

Interrupt / resume demo sketch:

- [interrupt_resume_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino)

Important behavior:

- the dashboard still works with the legacy manual sketch
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

The new `Home` page is the unified UI entry.

What it does:

- presents the whole UI as one navigation system
- shows current bridge, robot, mode, auto-stage, and task-selection snapshot
- links to all major feature areas
- avoids forcing users to edit URLs manually

What it does not do:

- it does not replace the combined dashboard
- it does not add a new frontend framework
- it does not change the current control logic

## Dashboard Controls

The `/dashboard` page includes:

- a `Task Selection` placeholder module
  - `Full Mission`
  - `Obstacle Mission`
  - `Pickup Mission`
  - `Delivery Mission`
  - `Manual Test`
- placeholder task actions
  - `Start Task`
  - `Stop Task / Cancel`
  - `Resume Task`
- placeholder task fields
  - `Selected Task`
  - `Task Description`
  - `Task Status`
  - `Current Stage`
  - `Resume Available`
  - `Last Saved Stage`
  - `Obstacle Interrupt`
  - `Obstacle Hook`
- interrupt / resume demo buttons
  - `Start Auto Task`
  - `Take Manual Control`
  - `Return To Auto / Resume Task`
  - `Cancel Resume / Abort Task`
- dashboard navigation links back to:
  - `Home`
  - `Telemetry`
  - `Manual`
  - `Tasks`
  - `Arm`
  - `Status / Debug`
  - `Unlock`

Manual control gating:

- if the interrupt/resume protocol is active, drive and arm commands are blocked until `MANUAL_OVERRIDE` is entered
- if the old manual-only sketch is connected, the dashboard keeps legacy direct manual behavior
- if manual mode no longer holds a saved task, `Start Auto Task` becomes available again and starts a fresh auto run

## Task Selection Placeholder

The task-selection block is intentionally separate from the current interrupt/resume demo.

It only reserves UI and HTTP contracts for the future proposal task runtime.

What it does now:

- lets the user choose one of five mission presets
- shows task description and placeholder task state
- exposes `start / stop / resume` buttons for UI validation
- keeps a mock `savedStage` / `resumeAvailable` flow so the resume UI can be tested before the backend exists

What it does not do yet:

- it does not start the current Arduino auto demo
- it does not send serial task commands
- it does not own the real Jetson or Arduino task state machine

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

Frontend helper functions are exposed on `window.taskSelectionApi`:

- `selectTask(taskId)`
- `startSelectedTask()`
- `stopSelectedTask()`
- `resumeSelectedTask()`
- `getTaskSelectionState()`
- `loadTaskSelectionState()`
- `updateTaskStageDisplay(stage)`
- `setTaskSelectionStatus(status, message)`
- `markObstacleInterruptActive(active)`
- `markResumeAvailable(resumeAvailable, savedStage)`

These helpers currently operate on placeholder state only, but their names and payload shapes are meant to survive the later backend wiring step.

## HTTP APIs

Existing APIs:

- `POST /api/control`
- `POST /api/arm`
- `POST /api/unlock`

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
  "source": "dashboard-task-selection"
}
```

`POST /api/task-selection/mock-update`

```json
{
  "taskStatus": "RUNNING_PLACEHOLDER",
  "currentStage": "RFID_PICKUP_WAIT",
  "savedStage": "CORRIDOR_DRIVE",
  "resumeAvailable": true,
  "obstacleInterruptActive": false,
  "source": "future-runtime"
}
```

Why `mock-update` exists:

- it gives the UI a stable future hook for runtime-to-dashboard sync
- it is still placeholder-only today
- later the Jetson task manager can call this route or replace its implementation directly inside `server.py`

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

- the dashboard still receives live updates
- the home page still shows a live snapshot
- the task-selection placeholder module also keeps its state in memory
- interrupt / resume controls are visible and enabled
- manual control gating behaves like the real interrupt/resume protocol
- `OPEN_DOOR` still triggers the simulated unlock state

Mock mode is still only for UI / API validation.

It is not a motor-control simulation.

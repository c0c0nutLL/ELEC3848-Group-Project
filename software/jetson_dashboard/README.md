# Jetson Dashboard

This directory contains the current Jetson Nano web dashboard and serial bridge.

Current path:

- `Arduino -> USB serial -> Jetson Nano -> browser`
- `Browser -> Jetson HTTP API -> USB serial -> Arduino`
- `Phone -> /unlock -> password verify -> OPEN_DOOR -> Arduino`

## Main Files

- [server.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/server.py)
  - main Jetson service
  - reads Arduino JSON telemetry over serial
  - serves the dashboard, unlock page, and HTTP APIs
  - now also handles task start / manual override / resume / abort actions
- [dashboard.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/dashboard.html)
  - live status page
  - auto-task interrupt / resume controls
  - manual drive controls
  - PWM slider
  - arm controls
- [unlock.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock.html)
  - unlock page for phone access
- [unlock_config.json](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock_config.json)
  - fixed password and unlock command config

## Paired Arduino Sketches

Legacy manual-only sketch:

- [telemetry_serial_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino)

Interrupt / resume demo sketch:

- [interrupt_resume_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino)

Important behavior:

- the dashboard still works with the legacy manual sketch
- but interrupt / resume buttons stay unavailable until the Arduino telemetry includes `mode_protocol_version`
- the new interrupt / resume workflow therefore requires `interrupt_resume_demo.ino`

## New Runtime Model

The dashboard now expects two layers of state when the interrupt / resume demo sketch is running:

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

## UI Controls Added

The root page `/` now includes:

- `Start Auto Task`
- `Take Manual Control`
- `Return To Auto / Resume Task`
- `Cancel Resume / Abort Task`

The status panel now also shows:

- `Current Mode`
- `Auto Stage`
- `Saved Stage`
- `Resume Pending`
- `Task Active`
- `Resume Note`

Manual control gating:

- if the interrupt/resume protocol is active, drive and arm commands are blocked until `MANUAL_OVERRIDE` is entered
- if the old manual-only sketch is connected, the dashboard keeps legacy direct manual behavior
- if manual mode no longer holds a saved task, `Start Auto Task` becomes available again and starts a fresh auto run

## HTTP APIs

Existing APIs:

- `POST /api/control`
- `POST /api/arm`
- `POST /api/unlock`

New APIs:

- `POST /api/task`
- `POST /api/mode`

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

Other mode actions:

- `RESUME_AUTO`
- `ABORT_TASK`

Task actions:

- `START`
- `ABORT`

## Serial Protocol

Jetson sends these task-level commands to the new demo sketch:

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

Telemetry fields added by the new sketch:

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

## Resume Rules

The current implementation is honest about recovery limits.

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
- realtime commands are rejected while the serial bridge is disconnected
- queued commands are dropped when the serial bridge disconnects

Local access:

- Dashboard: `http://127.0.0.1:8080/`
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
- interrupt / resume controls are visible and enabled
- manual control gating behaves like the real interrupt/resume protocol
- `OPEN_DOOR` still triggers the simulated unlock state

Mock mode is still only for UI / API validation.

It is not a motor-control simulation.

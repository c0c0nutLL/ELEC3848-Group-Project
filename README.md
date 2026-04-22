# ELEC3848 Group Project

This repository contains the current ELEC3848 AMR hardware, Arduino runtime, Jetson Nano dashboard, and project planning files.

Project-wide context files:

- `plan.md`
- `agent.md`

## Current Authoritative Runtime Paths

Arduino runtime source of truth:

- `firmware/arduino/proposal_amr/final_integrated_firmware/final_integrated_firmware.ino`

Jetson UI and serial bridge source of truth:

- `software/jetson_dashboard/server.py`
- `software/jetson_dashboard/tasks.html`
- `software/jetson_dashboard/manual.html`
- `software/jetson_dashboard/status.html`
- `software/jetson_dashboard/debug.html`
- `software/jetson_dashboard/unlock.html`

## System Architecture

Jetson Nano responsibilities:

- serve the web UI
- expose HTTP APIs
- enforce the LAN manual-access password gate
- handle the unlock password flow
- bridge browser commands to Arduino over USB serial
- mirror Arduino telemetry to browser clients

Arduino responsibilities:

- own the mission state machine
- own run mode (`AUTO`, `MANUAL_OVERRIDE`, `FAULT`)
- own task state
- own motor, arm, cargo-door, and RFID handling
- publish telemetry JSON to Jetson

Jetson does not own a second autonomous task state machine.

## Canonical Tasks

- `FULL_MISSION`
- `PICKUP_MISSION`
- `DELIVERY_MISSION`
- `MANUAL_TEST`

## Hardware Notes

Current navigation and sensing assumptions:

- front ultrasonic pair remains active
- side ultrasonics are not part of the current runtime model
- RFID is connected directly to Arduino
- QR/password unlock remains on Jetson

Pin mapping source of truth:

- `hardware/amr_pin_mapping_template.xlsx`

If hardware pin assignments change, update the workbook first and then update the corresponding Arduino runtime and planning files.

## Key Directories

- `firmware/arduino/proposal_amr/`
  - proposal-stage Arduino work
  - includes historical demos and the final integrated runtime
- `software/jetson_dashboard/`
  - Jetson Nano UI, HTTP API, and serial bridge
- `hardware/`
  - hardware planning assets including the pin mapping workbook
- `software/jetson_yolo_tests/`
  - Jetson-side YOLO experiments and related notes

## Historical Sketches

Older sketches under `proposal_amr/` are kept as references and experiments. They are not the authoritative runtime once `final_integrated_firmware.ino` is in use.

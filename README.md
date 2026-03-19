# ELEC3848 AMR Project

This repository is for the ELEC3848 Integrated Design Project.

Our project is an indoor Autonomous Mobile Robot (AMR) for smart delivery. The system is intended to combine motion control, docking, station verification, cargo release, telemetry upload, and dashboard visualization into one delivery workflow.

## Project Goal

Build an AMR that can:

- move to a target station
- approach and dock safely
- verify the destination
- release the carried item
- upload status and sensor data
- display task status on a computer dashboard

## Planned Modules

- Motion control
- Docking and safety sensing
- Station verification
- Cargo release
- Telemetry upload
- Dashboard and task log

## Repository Structure

Suggested structure:

```text
.
|-- README.md
|-- docs/
|-- firmware/
|-- hardware/
|-- software/
|-- tests/
`-- .gitignore
```

## Notes

- Large reference documents are kept outside version control.
- Local virtual environments are ignored.
- Source code, wiring notes, setup steps, and test records should be committed here.

## Team Workflow

1. Create a branch for each task.
2. Commit small, clear changes.
3. Push to GitHub regularly.
4. Open pull requests for review when needed.

## Next Steps

- add Arduino control code under `firmware/`
- add Jetson Nano or PC-side software under `software/`
- add wiring diagrams and hardware notes under `hardware/`
- add test logs and validation results under `tests/`

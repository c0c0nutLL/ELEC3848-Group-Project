# Full Mission Component Tests

This directory is reserved for staged Arduino-side development of the final pickup + delivery mission.

It exists for the items that should be tested separately before they are merged into the final integrated firmware:

- mission state-machine structure
- turnaround behavior
- pickup sequence behavior
- end-to-end full mission flow behavior

Current project decisions reflected here:

- RFID is the main station-verification method
- `Full mission` contains both pickup and delivery
- `Pickup mission` and `Delivery mission` may be called separately as subsets
- QR + fixed password is part of the final delivery workflow
- cargo-door actuation uses a dedicated door servo separate from the arm
- the door-servo pin is still pending confirmation

This directory is not the final integrated firmware location.

Use [final_integrated_firmware](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\firmware\arduino\proposal_amr\final_integrated_firmware) for the eventual all-in-one sketch.

## Included test areas

- [mission_state_machine_sandbox](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\firmware\arduino\proposal_amr\full_mission_component_tests\mission_state_machine_sandbox)
  - stage-name and flow sandbox
- [turn_around_test](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\firmware\arduino\proposal_amr\full_mission_component_tests\turn_around_test)
  - dedicated chassis turnaround bench test
- [pickup_sequence_test](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\firmware\arduino\proposal_amr\full_mission_component_tests\pickup_sequence_test)
  - dedicated arm-side pickup sequence bench test
- [full_mission_flow_sandbox](F:\鏉庝粊鏉癨棣欐腐澶у\2025-2026\ELEC3848\Group project\firmware\arduino\proposal_amr\full_mission_component_tests\full_mission_flow_sandbox)
  - end-to-end sandbox for pickup + delivery flow, door-close timing, and obstacle insertions

## Current LED Hook

Task-status LED logic is currently attached to `full_mission_flow_sandbox`.

Reason:

- it is the closest Arduino-side sketch to the planned full pickup + delivery mission
- it already contains both:
  - obstacle-interrupt behavior
  - delivery-side `WAIT_USER_UNLOCK`

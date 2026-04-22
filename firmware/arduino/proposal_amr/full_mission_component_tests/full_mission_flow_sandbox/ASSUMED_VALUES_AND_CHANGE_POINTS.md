# Assumed Values And Change Points

Target sketch:

- `firmware/arduino/proposal_amr/full_mission_component_tests/full_mission_flow_sandbox/full_mission_flow_sandbox.ino`

This file lists every placeholder value in the sandbox sketch that still needs to be replaced by a tested real value.

Only the sandbox sketch is tracked here. If you later change the delivery auto-close time, also align the Jetson unlock config and the currently paired proposal sketches.

## Pins

- Line 38: `FRONT_RIGHT_TRIG_PIN = 6` -> final right-front ultrasonic `TRIG` pin.
- Line 39: `FRONT_RIGHT_ECHO_PIN = 4` -> final right-front ultrasonic `ECHO` pin.
- Line 40: `FRONT_LEFT_TRIG_PIN = 10` -> final left-front ultrasonic `TRIG` pin.
- Line 41: `FRONT_LEFT_ECHO_PIN = 7` -> final left-front ultrasonic `ECHO` pin.
- Line 42: `ARM_GRIP_SERVO_PIN = 44` -> final arm gripper servo signal pin.
- Line 43: `ARM_LIFT_SERVO_PIN = 45` -> final arm lift servo signal pin.
- Line 44: `DOOR_SERVO_PIN = 46` -> final dedicated cargo-door servo signal pin.
- Line 45: `RFID_SS_PIN = 53` -> final RFID SPI chip-select pin.
- Line 46: `RFID_RST_PIN = 49` -> final RFID reset pin.
- Line 47: `LOCAL_START_BUTTON_PIN = 33` -> optional local full-mission start button pin for bench testing.
- Line 48: `LOCAL_CLOSE_BUTTON_PIN = 32` -> optional local close-door button pin for bench testing.

## RFID / Mission Identity

- Line 50: `PICKUP_RFID_TAG = "PICKUP_TAG_01"` -> real pickup-station RFID tag or UID text.
- Line 51: `DELIVERY_RFID_TAG = "DELIVERY_TAG_01"` -> real delivery-station RFID tag or UID text.

## Motor PWM And Servo Angles

- Line 53: `DEFAULT_TRAVEL_PWM = 92` -> tested straight-drive PWM for mission travel.
- Line 54: `DEFAULT_STRAFE_PWM = 88` -> tested strafe PWM for obstacle detour.
- Line 55: `DEFAULT_TURN_PWM = 96` -> tested PWM for the turnaround segment.
- Line 56: `DOOR_CLOSED_DEG = 18` -> tested cargo-door closed angle.
- Line 57: `DOOR_OPEN_DEG = 108` -> tested cargo-door open angle.
- Line 58: `ARM_HOME_LIFT_DEG = 110` -> tested arm home lift angle.
- Line 59: `ARM_HOME_GRIP_DEG = 90` -> tested arm home grip angle.
- Line 60: `ARM_PICK_LIFT_DEG = 72` -> tested lift angle for pickup reach.
- Line 61: `ARM_OPEN_GRIP_DEG = 120` -> tested gripper-open angle.
- Line 62: `ARM_CLOSE_GRIP_DEG = 60` -> tested gripper-close angle.

## Timing

- Line 64: `START_SETTLE_MS = 500UL` -> settle delay after mission start.
- Line 65: `PICKUP_STEP_WAIT_MS = 600UL` -> dwell time between arm pickup poses.
- Line 66: `LOAD_SETTLE_MS = 2500UL` -> load placement dwell time before door close.
- Line 67: `DOOR_LOAD_SETTLE_MS = 1200UL` -> wait time after opening the load door.
- Line 68: `TURN_AROUND_MS = 720UL` -> timed 180-degree turnaround duration.
- Line 69: `DELIVERY_DOOR_AUTO_CLOSE_MS = 60000UL` -> delivery-side auto-close timeout.
- Line 70: `STATUS_INTERVAL_MS = 1000UL` -> serial status-report period.
- Line 71: `OBSTACLE_STOP_HOLD_MS = 400UL` -> pause time before obstacle detour starts.
- Line 72: `OBSTACLE_STRAFE_MS = 850UL` -> sideways move duration for obstacle detour.
- Line 73: `OBSTACLE_PASS_MS = 1300UL` -> forward pass duration inside obstacle detour.
- Line 74: `OBSTACLE_RETURN_MS = 850UL` -> return-strafe duration after obstacle pass.

## Obstacle Thresholds And Sim Defaults

- Line 76: `OBSTACLE_TRIGGER_CM = 45.0f` -> obstacle trigger distance threshold.
- Line 77: `OBSTACLE_CLEAR_CM = 65.0f` -> distance considered clear again after detour.
- Line 78: `DEFAULT_SIM_FRONT_CM = 120.0f` -> idle simulated front distance used by the sandbox.
- Line 79: `DEFAULT_OBSTACLE_TEST_CM = 30.0f` -> default obstacle test distance when the command omits a value.
- Line 80: `IDLE_FRONT_MARGIN_CM = 20.0f` -> extra idle margin applied above the clear threshold.
- Line 81: `CLEAR_FRONT_MARGIN_CM = 10.0f` -> extra margin restored after clearing an obstacle.

## Motion Trim

- Line 82: `FORWARD_LEFT_TRIM = 1.08f` -> left-side forward compensation factor.
- Line 83: `STRAFE_LEFT_RF_TRIM = 1.05f` -> right-front compensation factor for left strafe.
- Line 84: `STRAFE_RIGHT_RF_TRIM = 1.10f` -> right-front compensation factor for right strafe.
- Line 85: `STRAFE_RIGHT_LR_TRIM = 1.10f` -> left-rear compensation factor for right strafe.

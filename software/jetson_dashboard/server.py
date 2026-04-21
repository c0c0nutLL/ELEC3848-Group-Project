import argparse
import ipaddress
import json
import queue
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from socketserver import ThreadingMixIn
from typing import Any, Deque, Dict, List, Optional, Tuple
from urllib.parse import urlparse

try:
    import serial  # type: ignore
except ImportError:
    serial = None


BASE_DIR = Path(__file__).resolve().parent
DASHBOARD_HTML = BASE_DIR / "dashboard.html"
UNLOCK_HTML = BASE_DIR / "unlock.html"
UNLOCK_CONFIG_PATH = BASE_DIR / "unlock_config.json"

DEFAULT_UNLOCK_CONFIG = {
    "robot_id": "amr01",
    "unlock_mode": "fixed",
    "fixed_password": "3848",
    "door_command": "OPEN_DOOR",
    "unlock_cooldown_ms": 5000,
}

ALLOWED_DRIVE_DIRECTIONS = {"STOP", "FORWARD", "BACKWARD", "LEFT", "RIGHT"}
ALLOWED_ARM_JOINTS = {"LIFT", "GRIP"}
ALLOWED_MODE_ACTIONS = {"TAKE_MANUAL", "RESUME_AUTO", "ABORT_TASK"}
ALLOWED_TASK_ACTIONS = {"START", "ABORT"}
DEFAULT_DRIVE_POWER = 96
MAX_DRIVE_POWER = 255
MIN_ARM_ANGLE_DEG = 0
MAX_ARM_ANGLE_DEG = 180
DEFAULT_ARM_LIFT_DEG = 110
DEFAULT_ARM_GRIP_DEG = 90
ARM_ACTIVE_WINDOW_MS = 1200
MOCK_UNLOCK_OPEN_MS = 5000
MOCK_DISTANCE_MIN_CM = 8.0
MOCK_DISTANCE_MAX_CM = 150.0
MODE_PROTOCOL_VERSION = "interrupt-resume-v1"
RUN_MODE_AUTO = "AUTO"
RUN_MODE_MANUAL_OVERRIDE = "MANUAL_OVERRIDE"
RUN_MODE_FAULT = "FAULT"

state_lock = threading.Lock()
clients_lock = threading.Lock()
unlock_lock = threading.Lock()
control_lock = threading.Lock()
arm_lock = threading.Lock()

latest_state = {}  # type: Dict[str, Any]
state_history = deque(maxlen=100)  # type: Deque[Dict[str, Any]]
unlock_history = deque(maxlen=50)  # type: Deque[Dict[str, Any]]
client_queues = []  # type: List[queue.Queue]
serial_command_queue = queue.Queue(maxsize=20)
stop_event = threading.Event()

unlock_config = dict(DEFAULT_UNLOCK_CONFIG)  # type: Dict[str, Any]
bridge_mode = "serial"
last_unlock_sent_ms = 0
mock_unlock_until_ms = 0
control_state = {
    "direction": "STOP",
    "power": DEFAULT_DRIVE_POWER,
    "status": "IDLE",
    "source": "server",
    "last_command_ms": 0,
}  # type: Dict[str, Any]
arm_state = {
    "lift_deg": DEFAULT_ARM_LIFT_DEG,
    "grip_deg": DEFAULT_ARM_GRIP_DEG,
    "status": "IDLE",
    "source": "server",
    "last_command_ms": 0,
}  # type: Dict[str, Any]
mock_sensor_state = {
    "right_cm": 92.0,
    "left_cm": 89.0,
}  # type: Dict[str, float]


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def current_time_ms() -> int:
    return int(time.time() * 1000)


def load_unlock_config(config_path: Path) -> Dict[str, Any]:
    config = dict(DEFAULT_UNLOCK_CONFIG)
    if config_path.exists():
        loaded = json.loads(config_path.read_text(encoding="utf-8"))
        if isinstance(loaded, dict):
            config.update(loaded)
    return config


def is_loopback_client(client_ip: str) -> bool:
    try:
        return ipaddress.ip_address(client_ip).is_loopback
    except ValueError:
        return False


def bridge_is_connected() -> bool:
    if bridge_mode == "mock":
        return True

    with state_lock:
        return str(latest_state.get("bridge_status", "")) == "CONNECTED"


def clear_serial_command_queue() -> int:
    dropped = 0
    while True:
        try:
            serial_command_queue.get_nowait()
            dropped += 1
        except queue.Empty:
            return dropped


def clamp(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, value))


def normalize_direction(raw_direction: Any) -> str:
    direction = str(raw_direction or "STOP").strip().upper()
    if direction not in ALLOWED_DRIVE_DIRECTIONS:
        raise ValueError(f"direction must be one of {sorted(ALLOWED_DRIVE_DIRECTIONS)}")
    return direction


def normalize_power(raw_power: Any) -> int:
    try:
        power = int(raw_power)
    except (TypeError, ValueError):
        raise ValueError("power must be an integer") from None
    return clamp(power, 0, MAX_DRIVE_POWER)


def normalize_arm_joint(raw_joint: Any) -> str:
    joint = str(raw_joint or "").strip().upper()
    if joint not in ALLOWED_ARM_JOINTS:
        raise ValueError(f"joint must be one of {sorted(ALLOWED_ARM_JOINTS)}")
    return joint


def normalize_arm_angle(raw_angle: Any) -> int:
    try:
        angle = int(raw_angle)
    except (TypeError, ValueError):
        raise ValueError("angle must be an integer") from None
    return clamp(angle, MIN_ARM_ANGLE_DEG, MAX_ARM_ANGLE_DEG)


def normalize_mode_action(raw_action: Any) -> str:
    action = str(raw_action or "").strip().upper()
    if action not in ALLOWED_MODE_ACTIONS:
        raise ValueError(f"action must be one of {sorted(ALLOWED_MODE_ACTIONS)}")
    return action


def normalize_task_action(raw_action: Any) -> str:
    action = str(raw_action or "").strip().upper()
    if action not in ALLOWED_TASK_ACTIONS:
        raise ValueError(f"action must be one of {sorted(ALLOWED_TASK_ACTIONS)}")
    return action


def get_state_snapshot() -> Dict[str, Any]:
    with state_lock:
        return dict(latest_state)


def mode_protocol_available(snapshot: Optional[Dict[str, Any]] = None) -> bool:
    if bridge_mode == "mock":
        return True

    if snapshot is None:
        snapshot = get_state_snapshot()
    version = snapshot.get("mode_protocol_version")
    return isinstance(version, str) and len(version) > 0


def current_mode(snapshot: Optional[Dict[str, Any]] = None) -> str:
    if snapshot is None:
        snapshot = get_state_snapshot()
    return str(snapshot.get("current_mode", "") or "").strip().upper()


def current_auto_stage(snapshot: Optional[Dict[str, Any]] = None) -> str:
    if snapshot is None:
        snapshot = get_state_snapshot()
    return str(snapshot.get("current_auto_stage", "") or "").strip().upper()


def saved_auto_stage(snapshot: Optional[Dict[str, Any]] = None) -> str:
    if snapshot is None:
        snapshot = get_state_snapshot()
    return str(snapshot.get("saved_auto_stage", "") or "").strip().upper()


def manual_control_allowed(snapshot: Optional[Dict[str, Any]] = None) -> Tuple[bool, str]:
    if snapshot is None:
        snapshot = get_state_snapshot()

    if not mode_protocol_available(snapshot):
        return True, ""

    mode = current_mode(snapshot)
    if mode == RUN_MODE_MANUAL_OVERRIDE:
        return True, ""
    if bool(snapshot.get("fault_latched")) or mode == RUN_MODE_FAULT:
        return False, "manual override is required, and resume is blocked until the fault is cleared"
    return False, "take manual control first"


def interrupt_resume_ready(snapshot: Optional[Dict[str, Any]] = None) -> bool:
    return mode_protocol_available(snapshot)


def require_interrupt_resume_protocol(snapshot: Optional[Dict[str, Any]] = None) -> Tuple[bool, str]:
    if interrupt_resume_ready(snapshot):
        return True, ""
    return (
        False,
        "current Arduino sketch does not expose interrupt/resume telemetry; flash firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino",
    )


def build_control_patch() -> Dict[str, Any]:
    with control_lock:
        return {
            "control_direction": control_state["direction"],
            "motor_pwm": control_state["power"],
            "control_status": control_state["status"],
            "control_source": control_state["source"],
            "control_last_command_ms": control_state["last_command_ms"] or None,
        }


def set_control_state(direction: str, power: int, status: str, source: str) -> Dict[str, Any]:
    now_ms = current_time_ms()
    with control_lock:
        control_state.update(
            {
                "direction": direction,
                "power": power,
                "status": status,
                "source": source,
                "last_command_ms": now_ms,
            }
        )
    return build_control_patch()


def build_arm_patch() -> Dict[str, Any]:
    with arm_lock:
        return {
            "arm_lift_deg": arm_state["lift_deg"],
            "arm_grip_deg": arm_state["grip_deg"],
            "arm_status": arm_state["status"],
            "arm_source": arm_state["source"],
            "arm_last_command_ms": arm_state["last_command_ms"] or None,
        }


def set_arm_state(joint: str, angle: int, status: str, source: str) -> Dict[str, Any]:
    now_ms = current_time_ms()
    with arm_lock:
        if joint == "LIFT":
            arm_state["lift_deg"] = angle
        elif joint == "GRIP":
            arm_state["grip_deg"] = angle
        arm_state["status"] = status
        arm_state["source"] = source
        arm_state["last_command_ms"] = now_ms
    return build_arm_patch()


def broadcast_state(packet: Dict[str, Any]) -> None:
    payload = json.dumps(packet, ensure_ascii=False)

    with clients_lock:
        queues_snapshot = list(client_queues)

    for client_queue in queues_snapshot:
        try:
            client_queue.put_nowait(payload)
        except queue.Full:
            try:
                client_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                client_queue.put_nowait(payload)
            except queue.Full:
                pass


def update_state(packet: Dict[str, Any]) -> None:
    packet["jetson_received_ms"] = current_time_ms()

    with state_lock:
        latest_state.update(packet)
        state_history.appendleft(dict(latest_state))
        snapshot = dict(latest_state)

    broadcast_state(snapshot)


def patch_state(patch: Dict[str, Any], add_history: bool = False) -> None:
    with state_lock:
        latest_state.update(patch)
        latest_state["jetson_received_ms"] = current_time_ms()
        if add_history:
            state_history.appendleft(dict(latest_state))
        snapshot = dict(latest_state)

    broadcast_state(snapshot)


def record_unlock_event(success: bool, reason: str, client_ip: str, command_sent: str = "") -> None:
    event = {
        "ts_ms": current_time_ms(),
        "robot_id": unlock_config.get("robot_id", "amr01"),
        "success": success,
        "reason": reason,
        "client_ip": client_ip,
        "command_sent": command_sent,
    }
    with unlock_lock:
        unlock_history.appendleft(event)


def queue_serial_command(command: str) -> Tuple[bool, str]:
    global mock_unlock_until_ms

    if bridge_mode == "mock":
        print(f"[serial-mock] command={command}")
        if command == str(unlock_config.get("door_command", "OPEN_DOOR")):
            mock_unlock_until_ms = current_time_ms() + MOCK_UNLOCK_OPEN_MS
        return True, ""

    if not bridge_is_connected():
        return False, "serial bridge is not connected"

    try:
        serial_command_queue.put_nowait(command)
        return True, ""
    except queue.Full:
        return False, "command queue is full"


def flush_serial_commands(ser: Any) -> None:
    while True:
        try:
            command = serial_command_queue.get_nowait()
        except queue.Empty:
            return

        payload = f"{command}\n".encode("utf-8")
        ser.write(payload)
        ser.flush()
        print(f"[serial] sent command: {command}")


def handle_serial_line(line: str) -> None:
    line = line.strip()
    if not line:
        return

    try:
        packet = json.loads(line)
    except json.JSONDecodeError:
        print(f"[serial] ignore non-json line: {line}")
        return

    if not isinstance(packet, dict):
        print("[serial] ignore non-object payload")
        return

    update_state(packet)
    print(
        "[serial] "
        f"state={packet.get('state', '?')} "
        f"mode={packet.get('current_mode', '?')} "
        f"auto={packet.get('current_auto_stage', '?')} "
        f"result={packet.get('delivery_result', '?')} "
        f"direction={packet.get('control_direction', '?')}"
    )


def trigger_manual_drive(
    direction_raw: Any,
    power_raw: Any,
    client_ip: str,
    source: str = "dashboard",
) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    snapshot = get_state_snapshot()
    allowed, block_reason = manual_control_allowed(snapshot)
    if not allowed:
        return False, {"ok": False, "message": block_reason}, HTTPStatus.CONFLICT
    protocol_active = mode_protocol_available(snapshot)

    try:
        direction = normalize_direction(direction_raw)
        power = normalize_power(power_raw)
    except ValueError as exc:
        return False, {"ok": False, "message": str(exc)}, HTTPStatus.BAD_REQUEST

    if direction != "STOP" and power == 0:
        direction = "STOP"

    command = f"DRIVE {direction} {power}"
    queued, queue_message = queue_serial_command(command)
    if not queued:
        return False, {"ok": False, "message": queue_message}, HTTPStatus.SERVICE_UNAVAILABLE

    status = "ACTIVE" if direction != "STOP" else "IDLE"
    message = "stop command queued" if direction == "STOP" else f"{direction.lower()} command queued"
    patch = {
        **set_control_state(direction, power, status, source),
        "state": "IDLE" if direction == "STOP" else f"MANUAL_{direction}",
        "delivery_result": "READY" if direction == "STOP" else "MANUAL_CONTROL",
        "control_message": message,
        "control_last_client": client_ip,
        "fault": "",
    }
    if protocol_active:
        patch.update(
            {
                "current_mode": RUN_MODE_MANUAL_OVERRIDE,
                "manual_session_active": True,
                "delivery_result": "RESUME_PENDING" if bool(snapshot.get("resume_pending")) else "MANUAL_CONTROL",
                "target_station": snapshot.get("target_station", "PROPOSAL_DEMO"),
            }
        )
    else:
        patch["target_station"] = "MANUAL"

    patch_state(
        patch,
        add_history=True,
    )

    return (
        True,
        {
            "ok": True,
            "message": message,
            "command": command,
            "direction": direction,
            "power": power,
        },
        HTTPStatus.OK,
    )


def trigger_arm_control(
    joint_raw: Any,
    angle_raw: Any,
    client_ip: str,
    source: str = "dashboard",
) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    snapshot = get_state_snapshot()
    allowed, block_reason = manual_control_allowed(snapshot)
    if not allowed:
        return False, {"ok": False, "message": block_reason}, HTTPStatus.CONFLICT
    protocol_active = mode_protocol_available(snapshot)

    try:
        joint = normalize_arm_joint(joint_raw)
        angle = normalize_arm_angle(angle_raw)
    except ValueError as exc:
        return False, {"ok": False, "message": str(exc)}, HTTPStatus.BAD_REQUEST

    command = f"ARM {joint} {angle}"
    queued, queue_message = queue_serial_command(command)
    if not queued:
        return False, {"ok": False, "message": queue_message}, HTTPStatus.SERVICE_UNAVAILABLE

    patch = {
        **set_arm_state(joint, angle, "ACTIVE", source),
        "arm_message": f"{joint.lower()} set to {angle} deg",
        "arm_last_client": client_ip,
        "fault": "",
    }
    if protocol_active:
        patch.update(
            {
                "current_mode": RUN_MODE_MANUAL_OVERRIDE,
                "manual_session_active": True,
            }
        )

    patch_state(patch, add_history=True)

    return (
        True,
        {
            "ok": True,
            "message": f"{joint.lower()} set to {angle} deg",
            "command": command,
            "joint": joint,
            "angle": angle,
        },
        HTTPStatus.OK,
    )


def trigger_task_action(
    action_raw: Any,
    client_ip: str,
    source: str = "dashboard",
) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    try:
        action = normalize_task_action(action_raw)
    except ValueError as exc:
        return False, {"ok": False, "message": str(exc)}, HTTPStatus.BAD_REQUEST

    snapshot = get_state_snapshot()
    protocol_ok, protocol_message = require_interrupt_resume_protocol(snapshot)
    if not protocol_ok:
        return False, {"ok": False, "message": protocol_message}, HTTPStatus.CONFLICT

    mode = current_mode(snapshot)
    stage = current_auto_stage(snapshot)
    saved_stage = saved_auto_stage(snapshot)
    resume_flag = bool(snapshot.get("resume_pending"))
    task_active = bool(snapshot.get("task_active"))

    if action == "START":
        if mode == RUN_MODE_MANUAL_OVERRIDE and (task_active or resume_flag or bool(saved_stage)):
            return (
                False,
                {
                    "ok": False,
                    "message": "manual override is holding a saved task; abort or resume before starting a new auto task",
                },
                HTTPStatus.CONFLICT,
            )

        if task_active and stage not in {"", "IDLE", "DONE", "FAULT"}:
            return False, {"ok": False, "message": "an auto task is already running"}, HTTPStatus.CONFLICT

        command = "TASK START"
        queued, queue_message = queue_serial_command(command)
        if not queued:
            return False, {"ok": False, "message": queue_message}, HTTPStatus.SERVICE_UNAVAILABLE

        patch_state(
            {
                "mode_protocol_version": MODE_PROTOCOL_VERSION,
                "current_mode": RUN_MODE_AUTO,
                "current_auto_stage": "START",
                "saved_auto_stage": "",
                "resume_pending": False,
                "manual_session_active": False,
                "task_active": True,
                "fault_latched": False,
                "resume_note": "No saved task",
                "mode_message": "auto task start requested",
                "task_message": "auto task start requested",
                "task_last_client": client_ip,
                "task_last_source": source,
                "task_last_action": action,
                "task_last_command": command,
                "delivery_result": "AUTO_STARTING",
                "target_station": "PROPOSAL_DEMO",
                "fault": "",
            },
            add_history=True,
        )

        return (
            True,
            {
                "ok": True,
                "message": "auto task start requested",
                "command": command,
                "action": action,
            },
            HTTPStatus.OK,
        )

    if mode == RUN_MODE_MANUAL_OVERRIDE:
        return (
            False,
            {
                "ok": False,
                "message": "manual override is active; use the manual abort action if you want to clear the saved task and stay in manual mode",
            },
            HTTPStatus.CONFLICT,
        )

    command = "TASK ABORT"
    queued, queue_message = queue_serial_command(command)
    if not queued:
        return False, {"ok": False, "message": queue_message}, HTTPStatus.SERVICE_UNAVAILABLE

    patch_state(
        {
            "mode_protocol_version": MODE_PROTOCOL_VERSION,
            "current_mode": RUN_MODE_AUTO,
            "current_auto_stage": "IDLE",
            "saved_auto_stage": "",
            "resume_pending": False,
            "manual_session_active": False,
            "task_active": False,
            "fault_latched": False,
            "resume_note": "No saved task",
            "mode_message": "auto task aborted",
            "task_message": "auto task aborted",
            "task_last_client": client_ip,
            "task_last_source": source,
            "task_last_action": action,
            "task_last_command": command,
            "delivery_result": "READY",
            "fault": "",
        },
        add_history=True,
    )

    return (
        True,
        {
            "ok": True,
            "message": "auto task aborted",
            "command": command,
            "action": action,
        },
        HTTPStatus.OK,
    )


def trigger_mode_action(
    action_raw: Any,
    client_ip: str,
    source: str = "dashboard",
) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    try:
        action = normalize_mode_action(action_raw)
    except ValueError as exc:
        return False, {"ok": False, "message": str(exc)}, HTTPStatus.BAD_REQUEST

    snapshot = get_state_snapshot()
    protocol_ok, protocol_message = require_interrupt_resume_protocol(snapshot)
    if not protocol_ok:
        return False, {"ok": False, "message": protocol_message}, HTTPStatus.CONFLICT

    mode = current_mode(snapshot)
    stage = current_auto_stage(snapshot)
    saved_stage = saved_auto_stage(snapshot)
    task_active = bool(snapshot.get("task_active"))
    resume_flag = bool(snapshot.get("resume_pending"))
    fault_latched = bool(snapshot.get("fault_latched"))

    if action == "TAKE_MANUAL":
        if mode == RUN_MODE_MANUAL_OVERRIDE:
            return (
                True,
                {
                    "ok": True,
                    "message": "manual override is already active",
                    "action": action,
                    "saved_auto_stage": saved_stage,
                    "resume_pending": resume_flag,
                },
                HTTPStatus.OK,
            )

        command = "OVERRIDE TAKE"
        queued, queue_message = queue_serial_command(command)
        if not queued:
            return False, {"ok": False, "message": queue_message}, HTTPStatus.SERVICE_UNAVAILABLE

        guessed_saved_stage = saved_stage
        if not guessed_saved_stage and task_active and stage not in {"", "IDLE", "DONE", "FAULT"}:
            guessed_saved_stage = stage

        resume_pending_value = resume_flag or bool(guessed_saved_stage)
        message = "manual override requested"
        if fault_latched:
            message = "manual override requested; resume will stay blocked until the fault is cleared"
        elif not resume_pending_value:
            message = "manual override requested; there is no resumable auto stage yet"

        patch_state(
            {
                "mode_protocol_version": MODE_PROTOCOL_VERSION,
                "current_mode": RUN_MODE_MANUAL_OVERRIDE,
                "saved_auto_stage": guessed_saved_stage,
                "resume_pending": resume_pending_value,
                "manual_session_active": True,
                "task_active": task_active or resume_pending_value,
                "mode_message": message,
                "mode_last_client": client_ip,
                "mode_last_source": source,
                "mode_last_action": action,
                "mode_last_command": command,
                "resume_note": "Resume from saved stage" if resume_pending_value else "No saved task",
                "state": "MANUAL_OVERRIDE",
                "delivery_result": "RESUME_PENDING" if resume_pending_value else "MANUAL_ONLY",
                "control_direction": "STOP",
                "control_status": "IDLE",
                "fault": snapshot.get("fault", ""),
            },
            add_history=True,
        )

        return (
            True,
            {
                "ok": True,
                "message": message,
                "command": command,
                "action": action,
                "saved_auto_stage": guessed_saved_stage,
                "resume_pending": resume_pending_value,
            },
            HTTPStatus.OK,
        )

    if action == "RESUME_AUTO":
        if mode != RUN_MODE_MANUAL_OVERRIDE:
            return False, {"ok": False, "message": "system is not in manual override"}, HTTPStatus.CONFLICT

        if fault_latched or mode == RUN_MODE_FAULT:
            return (
                False,
                {"ok": False, "message": "fault is latched; abort or restart the task instead of resuming"},
                HTTPStatus.CONFLICT,
            )

        if not resume_flag or not saved_stage:
            return False, {"ok": False, "message": "no saved auto stage is available for resume"}, HTTPStatus.CONFLICT

        command = "OVERRIDE RESUME"
        queued, queue_message = queue_serial_command(command)
        if not queued:
            return False, {"ok": False, "message": queue_message}, HTTPStatus.SERVICE_UNAVAILABLE

        patch_state(
            {
                "mode_protocol_version": MODE_PROTOCOL_VERSION,
                "current_mode": RUN_MODE_AUTO,
                "current_auto_stage": saved_stage,
                "resume_pending": False,
                "manual_session_active": False,
                "task_active": True,
                "mode_message": "resume requested",
                "mode_last_client": client_ip,
                "mode_last_source": source,
                "mode_last_action": action,
                "mode_last_command": command,
                "resume_note": "Resume requested",
                "state": saved_stage,
                "delivery_result": "AUTO_RESUMING",
            },
            add_history=True,
        )

        return (
            True,
            {
                "ok": True,
                "message": "resume requested",
                "command": command,
                "action": action,
                "resume_stage": saved_stage,
            },
            HTTPStatus.OK,
        )

    if mode != RUN_MODE_MANUAL_OVERRIDE:
        return trigger_task_action("ABORT", client_ip, source=source)

    command = "OVERRIDE ABORT"
    queued, queue_message = queue_serial_command(command)
    if not queued:
        return False, {"ok": False, "message": queue_message}, HTTPStatus.SERVICE_UNAVAILABLE

    patch_state(
        {
            "mode_protocol_version": MODE_PROTOCOL_VERSION,
            "current_mode": RUN_MODE_MANUAL_OVERRIDE,
            "current_auto_stage": "IDLE",
            "saved_auto_stage": "",
            "resume_pending": False,
            "manual_session_active": True,
            "task_active": False,
            "mode_message": "saved task cleared; manual override remains active",
            "mode_last_client": client_ip,
            "mode_last_source": source,
            "mode_last_action": action,
            "mode_last_command": command,
            "resume_note": "No saved task",
            "state": "MANUAL_OVERRIDE",
            "delivery_result": "MANUAL_ONLY",
        },
        add_history=True,
    )

    return (
        True,
        {
            "ok": True,
            "message": "saved task cleared; manual override remains active",
            "command": command,
            "action": action,
        },
        HTTPStatus.OK,
    )


def serial_reader_loop(port: str, baudrate: int) -> None:
    if serial is None:
        raise RuntimeError("pyserial is not installed")

    while not stop_event.is_set():
        try:
            print(f"[serial] opening {port} @ {baudrate}")
            with serial.Serial(port, baudrate, timeout=0.2) as ser:
                patch_state({
                    "bridge_mode": "serial",
                    "bridge_status": "CONNECTED",
                    "serial_port": port,
                    "fault": "",
                })

                while not stop_event.is_set():
                    flush_serial_commands(ser)
                    raw_line = ser.readline()
                    if not raw_line:
                        continue
                    handle_serial_line(raw_line.decode("utf-8", errors="ignore"))
        except Exception as exc:
            print(f"[serial] disconnected: {exc}")
            dropped_commands = clear_serial_command_queue()
            if dropped_commands > 0:
                print(f"[serial] dropped {dropped_commands} queued command(s) after disconnect")
            update_state({
                "state": "BRIDGE_WAITING",
                "delivery_result": "SERIAL_DISCONNECTED",
                "bridge_mode": "serial",
                "bridge_status": "DISCONNECTED",
                "control_direction": "STOP",
                "control_status": "OFFLINE",
                "arm_lift_deg": None,
                "arm_grip_deg": None,
                "arm_status": "OFFLINE",
                "right_cm": None,
                "left_cm": None,
                "avg_cm": None,
                "diff_cm": None,
                "heading_deg": None,
                "mode_protocol_version": None,
                "current_mode": None,
                "current_auto_stage": None,
                "saved_auto_stage": None,
                "resume_pending": False,
                "manual_session_active": False,
                "task_active": False,
                "fault_latched": False,
                "resume_note": "",
                "fault": str(exc),
            })
            time.sleep(2)


def mock_reader_loop() -> None:
    start_ms = current_time_ms()
    while not stop_event.is_set():
        snapshot = get_state_snapshot()
        with control_lock:
            direction = str(control_state["direction"])
            power = int(control_state["power"])
            control_status = str(control_state["status"])
            control_source = str(control_state["source"])
            control_last_command_ms = int(control_state["last_command_ms"])
        with arm_lock:
            arm_lift_deg = int(arm_state["lift_deg"])
            arm_grip_deg = int(arm_state["grip_deg"])
            arm_source = str(arm_state["source"])
            arm_last_command_ms = int(arm_state["last_command_ms"])

        arm_status = "IDLE"
        if arm_last_command_ms > 0 and current_time_ms() - arm_last_command_ms < ARM_ACTIVE_WINDOW_MS:
            arm_status = "ACTIVE"

        right_cm = mock_sensor_state["right_cm"]
        left_cm = mock_sensor_state["left_cm"]
        step = max(0.5, power / 90.0)

        if direction == "FORWARD":
            right_cm = max(MOCK_DISTANCE_MIN_CM, right_cm - step)
            left_cm = max(MOCK_DISTANCE_MIN_CM, left_cm - step * 0.95)
        elif direction == "BACKWARD":
            right_cm = min(MOCK_DISTANCE_MAX_CM, right_cm + step)
            left_cm = min(MOCK_DISTANCE_MAX_CM, left_cm + step * 0.95)
        elif direction == "LEFT":
            right_cm = min(MOCK_DISTANCE_MAX_CM, right_cm + step * 0.35)
            left_cm = max(MOCK_DISTANCE_MIN_CM, left_cm - step * 0.35)
        elif direction == "RIGHT":
            right_cm = max(MOCK_DISTANCE_MIN_CM, right_cm - step * 0.35)
            left_cm = min(MOCK_DISTANCE_MAX_CM, left_cm + step * 0.35)

        mock_sensor_state["right_cm"] = right_cm
        mock_sensor_state["left_cm"] = left_cm

        cargo_door = "OPEN" if current_time_ms() < mock_unlock_until_ms else "CLOSED"
        current_mode_name = str(snapshot.get("current_mode", RUN_MODE_AUTO) or RUN_MODE_AUTO)
        current_stage_name = str(snapshot.get("current_auto_stage", "IDLE") or "IDLE")
        saved_stage_name = str(snapshot.get("saved_auto_stage", "") or "")
        target_station_name = str(snapshot.get("target_station", "MANUAL") or "MANUAL")
        resume_pending_value = bool(snapshot.get("resume_pending", False))
        task_active_value = bool(snapshot.get("task_active", False))
        manual_session_active_value = bool(snapshot.get("manual_session_active", current_mode_name == RUN_MODE_MANUAL_OVERRIDE))
        fault_latched_value = bool(snapshot.get("fault_latched", False))
        resume_note_value = str(snapshot.get("resume_note", "No saved task") or "No saved task")

        state = current_stage_name if current_stage_name else "IDLE"
        delivery_result = "READY"
        if current_mode_name == RUN_MODE_MANUAL_OVERRIDE:
            state = "MANUAL_OVERRIDE" if direction == "STOP" else f"MANUAL_{direction}"
            delivery_result = "RESUME_PENDING" if resume_pending_value else "MANUAL_ONLY"
        elif fault_latched_value:
            state = "FAULT"
            delivery_result = "FAULT"
        elif task_active_value:
            delivery_result = "AUTO_ACTIVE"
        elif cargo_door == "OPEN":
            state = "REMOTE_UNLOCK"
            delivery_result = "UNLOCKED"

        packet = {
            "ts_ms": current_time_ms() - start_ms,
            "state": state,
            "target_station": target_station_name,
            "right_cm": round(right_cm, 1),
            "left_cm": round(left_cm, 1),
            "avg_cm": round((right_cm + left_cm) * 0.5, 1),
            "diff_cm": round(right_cm - left_cm, 1),
            "heading_deg": 0.0,
            "rfid_tag": "",
            "rfid_match": False,
            "cargo_door": cargo_door,
            "fault": "",
            "bridge_mode": "mock",
            "bridge_status": "CONNECTED",
            "delivery_result": delivery_result,
            "mode_protocol_version": MODE_PROTOCOL_VERSION,
            "current_mode": current_mode_name,
            "current_auto_stage": current_stage_name,
            "saved_auto_stage": saved_stage_name,
            "resume_pending": resume_pending_value,
            "manual_session_active": manual_session_active_value,
            "task_active": task_active_value,
            "fault_latched": fault_latched_value,
            "resume_note": resume_note_value,
            "control_direction": direction,
            "motor_pwm": power,
            "control_status": control_status,
            "control_source": control_source,
            "control_last_command_ms": control_last_command_ms or None,
            "arm_lift_deg": arm_lift_deg,
            "arm_grip_deg": arm_grip_deg,
            "arm_status": arm_status,
            "arm_source": arm_source,
            "arm_last_command_ms": arm_last_command_ms or None,
        }
        update_state(packet)
        time.sleep(0.2)


def verify_unlock_password(password: str) -> Tuple[bool, str]:
    mode = str(unlock_config.get("unlock_mode", "fixed"))
    if mode != "fixed":
        return False, f"unlock mode {mode} is not supported yet"

    expected = str(unlock_config.get("fixed_password", ""))
    if password != expected:
        return False, "wrong password"

    return True, "password accepted"


def trigger_unlock(password: str, client_ip: str) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    global last_unlock_sent_ms

    ok, message = verify_unlock_password(password)
    if not ok:
        record_unlock_event(False, message, client_ip)
        patch_state({
            "unlock_status": "REJECTED",
            "unlock_message": message,
            "unlock_last_client": client_ip,
        })
        return False, {"ok": False, "message": message}, HTTPStatus.FORBIDDEN

    cooldown_ms = int(unlock_config.get("unlock_cooldown_ms", 0))
    now_ms = current_time_ms()
    command = str(unlock_config.get("door_command", "OPEN_DOOR"))
    event_success = False
    event_reason = ""
    event_command = ""
    state_patch = {}  # type: Dict[str, Any]
    response_payload = {}  # type: Dict[str, Any]
    response_status = HTTPStatus.OK

    with unlock_lock:
        if cooldown_ms > 0 and now_ms - last_unlock_sent_ms < cooldown_ms:
            retry_ms = cooldown_ms - (now_ms - last_unlock_sent_ms)
            message = f"cooldown active, retry in {retry_ms} ms"
            event_success = False
            event_reason = message
            state_patch = {
                "unlock_status": "COOLDOWN",
                "unlock_message": message,
                "unlock_last_client": client_ip,
            }
            response_payload = {"ok": False, "message": message}
            response_status = HTTPStatus.TOO_MANY_REQUESTS
        else:
            queued, queue_message = queue_serial_command(command)
            if not queued:
                message = queue_message
                event_success = False
                event_reason = message
                state_patch = {
                    "unlock_status": "QUEUE_FULL" if message == "command queue is full" else "BRIDGE_OFFLINE",
                    "unlock_message": message,
                    "unlock_last_client": client_ip,
                }
                response_payload = {"ok": False, "message": message}
                response_status = HTTPStatus.SERVICE_UNAVAILABLE
            else:
                last_unlock_sent_ms = now_ms
                event_success = True
                event_reason = "command queued"
                event_command = command
                state_patch = {
                    "unlock_status": "ACCEPTED",
                    "unlock_message": "fixed password accepted, unlock command sent",
                    "unlock_last_client": client_ip,
                    "unlock_last_command": command,
                    "unlock_last_ts_ms": now_ms,
                }
                response_payload = {"ok": True, "message": "unlock command sent", "command": command}
                response_status = HTTPStatus.OK

    record_unlock_event(event_success, event_reason, client_ip, command_sent=event_command)
    patch_state(state_patch)

    if event_success:
        return True, response_payload, response_status

    return False, response_payload, response_status


class DashboardHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, format: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)

        if parsed.path == "/":
            self.serve_file(DASHBOARD_HTML)
            return

        if parsed.path == "/unlock":
            self.serve_file(UNLOCK_HTML)
            return

        if parsed.path == "/api/state":
            self.serve_state()
            return

        if parsed.path == "/api/history":
            self.serve_history()
            return

        if parsed.path == "/api/unlock/history":
            self.serve_unlock_history()
            return

        if parsed.path == "/api/config":
            self.serve_public_config()
            return

        if parsed.path == "/events":
            self.serve_events()
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)

        if parsed.path == "/api/unlock":
            self.handle_unlock_post()
            return

        if parsed.path == "/api/control":
            self.handle_control_post()
            return

        if parsed.path == "/api/arm":
            self.handle_arm_post()
            return

        if parsed.path == "/api/mode":
            self.handle_mode_post()
            return

        if parsed.path == "/api/task":
            self.handle_task_post()
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def send_json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def serve_file(self, path: Path) -> None:
        raw = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def serve_state(self) -> None:
        with state_lock:
            payload = dict(latest_state)
        self.send_json(payload)

    def serve_history(self) -> None:
        with state_lock:
            payload = list(state_history)
        self.send_json(payload)

    def serve_unlock_history(self) -> None:
        with unlock_lock:
            payload = list(unlock_history)
        self.send_json(payload)

    def serve_public_config(self) -> None:
        payload = {
            "robot_id": unlock_config.get("robot_id", "amr01"),
            "unlock_mode": unlock_config.get("unlock_mode", "fixed"),
            "http_paths": {
                "dashboard": "/",
                "unlock": "/unlock",
            },
            "manual_control": {
                "api_path": "/api/control",
                "directions": sorted(ALLOWED_DRIVE_DIRECTIONS),
                "default_power": DEFAULT_DRIVE_POWER,
                "max_power": MAX_DRIVE_POWER,
                "loopback_only": True,
                "requires_manual_override_when_supported": True,
            },
            "arm_control": {
                "api_path": "/api/arm",
                "joints": sorted(ALLOWED_ARM_JOINTS),
                "min_angle_deg": MIN_ARM_ANGLE_DEG,
                "max_angle_deg": MAX_ARM_ANGLE_DEG,
                "default_lift_deg": DEFAULT_ARM_LIFT_DEG,
                "default_grip_deg": DEFAULT_ARM_GRIP_DEG,
                "step_deg": 5,
                "loopback_only": True,
            },
            "mode_control": {
                "api_path": "/api/mode",
                "actions": sorted(ALLOWED_MODE_ACTIONS),
                "protocol_version": MODE_PROTOCOL_VERSION,
                "loopback_only": True,
            },
            "task_control": {
                "api_path": "/api/task",
                "actions": sorted(ALLOWED_TASK_ACTIONS),
                "loopback_only": True,
            },
            "interrupt_resume": {
                "protocol_version": MODE_PROTOCOL_VERSION,
                "paired_sketch": "firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino",
                "required_state_fields": [
                    "current_mode",
                    "current_auto_stage",
                    "saved_auto_stage",
                    "resume_pending",
                ],
            },
        }
        self.send_json(payload)

    def serve_events(self) -> None:
        client_queue = queue.Queue(maxsize=20)
        with clients_lock:
            client_queues.append(client_queue)

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        try:
            with state_lock:
                if latest_state:
                    initial_payload = json.dumps(latest_state, ensure_ascii=False)
                    self.wfile.write(f"data: {initial_payload}\n\n".encode("utf-8"))
                    self.wfile.flush()

            while not stop_event.is_set():
                try:
                    payload = client_queue.get(timeout=15)
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                except queue.Empty:
                    self.wfile.write(b": keep-alive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            with clients_lock:
                if client_queue in client_queues:
                    client_queues.remove(client_queue)

    def read_json_body(self) -> Dict[str, Any]:
        content_length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(content_length) if content_length > 0 else b"{}"
        data = json.loads(body.decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("request body must be a JSON object")
        return data

    def handle_unlock_post(self) -> None:
        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        password = str(data.get("password", ""))
        client_ip = self.client_address[0] if self.client_address else "unknown"
        ok, payload, status = trigger_unlock(password, client_ip)
        if ok:
            self.send_json(payload, status)
        else:
            self.send_json(payload, status)

    def handle_control_post(self) -> None:
        client_ip = self.client_address[0] if self.client_address else "unknown"
        if not is_loopback_client(client_ip):
            self.send_json(
                {"ok": False, "message": "manual drive is only allowed from loopback clients"},
                HTTPStatus.FORBIDDEN,
            )
            return

        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        direction = data.get("direction", "STOP")
        power = data.get("power", DEFAULT_DRIVE_POWER)
        source = str(data.get("source", "dashboard"))
        ok, payload, status = trigger_manual_drive(direction, power, client_ip, source=source)
        self.send_json(payload, status)

    def handle_arm_post(self) -> None:
        client_ip = self.client_address[0] if self.client_address else "unknown"
        if not is_loopback_client(client_ip):
            self.send_json(
                {"ok": False, "message": "arm control is only allowed from loopback clients"},
                HTTPStatus.FORBIDDEN,
            )
            return

        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        joint = data.get("joint", "")
        angle = data.get("angle")
        source = str(data.get("source", "dashboard"))
        ok, payload, status = trigger_arm_control(joint, angle, client_ip, source=source)
        self.send_json(payload, status)

    def handle_mode_post(self) -> None:
        client_ip = self.client_address[0] if self.client_address else "unknown"
        if not is_loopback_client(client_ip):
            self.send_json(
                {"ok": False, "message": "mode control is only allowed from loopback clients"},
                HTTPStatus.FORBIDDEN,
            )
            return

        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        action = data.get("action", "")
        source = str(data.get("source", "dashboard"))
        ok, payload, status = trigger_mode_action(action, client_ip, source=source)
        self.send_json(payload, status)

    def handle_task_post(self) -> None:
        client_ip = self.client_address[0] if self.client_address else "unknown"
        if not is_loopback_client(client_ip):
            self.send_json(
                {"ok": False, "message": "task control is only allowed from loopback clients"},
                HTTPStatus.FORBIDDEN,
            )
            return

        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        action = data.get("action", "")
        source = str(data.get("source", "dashboard"))
        ok, payload, status = trigger_task_action(action, client_ip, source=source)
        self.send_json(payload, status)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Jetson Nano serial telemetry dashboard")
    parser.add_argument("--serial", default="/dev/ttyUSB0", help="Arduino serial device path")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP port")
    parser.add_argument("--mock", action="store_true", help="Use mock data instead of serial input")
    parser.add_argument("--config", default=str(UNLOCK_CONFIG_PATH), help="Unlock config JSON path")
    return parser.parse_args()


def main() -> None:
    global unlock_config, bridge_mode

    args = parse_args()
    unlock_config = load_unlock_config(Path(args.config))
    bridge_mode = "mock" if args.mock else "serial"

    if not args.mock and serial is None:
        raise SystemExit("pyserial is required. Install with: pip3 install -r requirements.txt")

    initial_state = {
        "bridge_mode": bridge_mode,
        "bridge_status": "STARTING",
        "robot_id": unlock_config.get("robot_id", "amr01"),
        "unlock_mode": unlock_config.get("unlock_mode", "fixed"),
        "unlock_status": "IDLE",
        "unlock_message": "",
        **build_control_patch(),
        **build_arm_patch(),
    }
    if args.mock:
        initial_state.update(
            {
                "mode_protocol_version": MODE_PROTOCOL_VERSION,
                "current_mode": RUN_MODE_AUTO,
                "current_auto_stage": "IDLE",
                "saved_auto_stage": "",
                "resume_pending": False,
                "manual_session_active": False,
                "task_active": False,
                "fault_latched": False,
                "resume_note": "No saved task",
            }
        )
    patch_state(initial_state)

    worker = threading.Thread(
        target=mock_reader_loop if args.mock else serial_reader_loop,
        args=() if args.mock else (args.serial, args.baud),
        daemon=True,
    )
    worker.start()

    server = ThreadedHTTPServer((args.host, args.http_port), DashboardHandler)
    print(f"[http] dashboard ready at http://{args.host}:{args.http_port}")
    print(f"[http] unlock page ready at http://{args.host}:{args.http_port}/unlock")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()

import argparse
import ipaddress
import json
import queue
import secrets
import threading
import time
from collections import deque
from http.cookies import SimpleCookie
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
HOME_HTML = BASE_DIR / "home.html"
DASHBOARD_HTML = BASE_DIR / "dashboard.html"
MANUAL_HTML = BASE_DIR / "manual.html"
TASKS_HTML = BASE_DIR / "tasks.html"
STATUS_HTML = BASE_DIR / "status.html"
DEBUG_HTML = BASE_DIR / "debug.html"
UNLOCK_HTML = BASE_DIR / "unlock.html"
MANUAL_ACCESS_HTML = BASE_DIR / "manual_access.html"
UNLOCK_CONFIG_PATH = BASE_DIR / "unlock_config.json"

DEFAULT_UNLOCK_CONFIG = {
    "robot_id": "amr01",
    "unlock_mode": "fixed",
    "fixed_password": "3848",
    "door_command": "OPEN_DOOR",
    "close_door_command": "CLOSE_DOOR",
    "door_auto_close_ms": 60000,
    "unlock_cooldown_ms": 5000,
}

ALLOWED_DRIVE_DIRECTIONS = {"STOP", "FORWARD", "BACKWARD", "LEFT", "RIGHT"}
ALLOWED_ARM_JOINTS = {"LIFT", "GRIP"}
ALLOWED_MODE_ACTIONS = {"TAKE_MANUAL", "RESUME_AUTO", "ABORT_TASK"}
ALLOWED_TASK_ACTIONS = {"START", "ABORT"}
TASK_SELECTION_TASKS = {
    "FULL_MISSION": {
        "label": "Full Mission",
        "name_zh": "全程任务",
        "summary": "Pickup + delivery main flow.",
        "description": (
            "Travel to the pickup RFID station, verify pickup, run the pickup sequence, "
            "open and close the cargo door for loading, turn around, travel to the delivery RFID station, "
            "then wait for QR + fixed-password unlock. After the unload door opens, the same page button "
            "becomes a close-door button, the 60-second close timer starts immediately, and manual close "
            "takes priority."
        ),
        "default_stage": "TRAVEL_TO_PICKUP",
        "supports_resume": True,
        "is_manual_test": False,
        "obstacle_interrupt_enabled": True,
    },
    "OBSTACLE_MISSION": {
        "label": "Obstacle Mission",
        "name_zh": "避障任务",
        "summary": "Obstacle interrupt path placeholder.",
        "description": (
            "Run obstacle detection and avoidance as a standalone validation path. "
            "In the final system this also acts as an interrupt path when YOLO detects a person or object "
            "and the distance sensor value is below the configured threshold."
        ),
        "default_stage": "OBSTACLE_MONITOR",
        "supports_resume": True,
        "is_manual_test": False,
        "obstacle_interrupt_enabled": True,
    },
    "PICKUP_MISSION": {
        "label": "Pickup Mission",
        "name_zh": "抓取任务",
        "summary": "Pickup half of full mission.",
        "description": (
            "Run the pickup-side flow only: travel to the pickup RFID station, verify pickup, run the pickup sequence, "
            "open the cargo door for loading, place the item, and close the door."
        ),
        "default_stage": "TRAVEL_TO_PICKUP",
        "supports_resume": True,
        "is_manual_test": False,
        "obstacle_interrupt_enabled": True,
    },
    "DELIVERY_MISSION": {
        "label": "Delivery Mission",
        "name_zh": "送货任务",
        "summary": "Delivery half of full mission.",
        "description": (
            "Run the delivery-side flow only: travel to the delivery RFID station, verify delivery, "
            "wait for QR + fixed-password unlock, then switch the UI button into a close-door button and start the "
            "60-second close timer immediately after the door opens. Manual close has priority over auto-close."
        ),
        "default_stage": "TRAVEL_TO_DELIVERY",
        "supports_resume": True,
        "is_manual_test": False,
        "obstacle_interrupt_enabled": True,
    },
    "MANUAL_TEST": {
        "label": "Manual Test",
        "name_zh": "人工测试",
        "summary": "UI-only manual chassis and arm operation.",
        "description": (
            "Do not run an automatic mission. Only allow manual drive and arm operations from the dashboard."
        ),
        "default_stage": "MANUAL_TEST_ONLY",
        "supports_resume": False,
        "is_manual_test": True,
        "obstacle_interrupt_enabled": False,
    },
}
TASK_SELECTION_STATUS_VALUES = ("IDLE", "START_REQUESTED", "STOPPED", "RESUME_REQUESTED")
TASK_SELECTION_STATE_FIELDS = [
    "selectedTask",
    "selectedTaskLabel",
    "selectedTaskName",
    "taskDescription",
    "taskSummary",
    "taskStatus",
    "currentStage",
    "resumeAvailable",
    "savedStage",
    "isManualTest",
    "isObstacleInterruptEnabled",
    "obstacleInterruptActive",
    "supportsResume",
    "placeholderEntryStage",
    "backendConnected",
    "backendBinding",
    "statusSource",
    "lastAction",
    "lastMessage",
    "lastUpdatedMs",
]
DEFAULT_TASK_SELECTION_TASK = "FULL_MISSION"
DEFAULT_DRIVE_POWER = 96
MAX_DRIVE_POWER = 255
MIN_ARM_ANGLE_DEG = 0
MAX_ARM_ANGLE_DEG = 180
DEFAULT_ARM_LIFT_DEG = 110
DEFAULT_ARM_GRIP_DEG = 90
ARM_ACTIVE_WINDOW_MS = 1200
MOCK_UNLOCK_OPEN_MS = 60000
MOCK_DISTANCE_MIN_CM = 8.0
MOCK_DISTANCE_MAX_CM = 150.0
MODE_PROTOCOL_VERSION = "interrupt-resume-v1"
RUN_MODE_AUTO = "AUTO"
RUN_MODE_MANUAL_OVERRIDE = "MANUAL_OVERRIDE"
RUN_MODE_FAULT = "FAULT"
MANUAL_ACCESS_PASSWORD = "elec3848"
MANUAL_ACCESS_COOKIE_NAME = "amr_manual_access"
MANUAL_ACCESS_SESSION_TTL_MS = 12 * 60 * 60 * 1000

state_lock = threading.Lock()
clients_lock = threading.Lock()
unlock_lock = threading.Lock()
control_lock = threading.Lock()
arm_lock = threading.Lock()
task_selection_lock = threading.Lock()
manual_access_lock = threading.Lock()

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
task_selection_state = {}  # type: Dict[str, Any]
manual_access_sessions = {}  # type: Dict[str, Dict[str, Any]]


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


def cleanup_manual_access_sessions(now_ms: Optional[int] = None) -> None:
    if now_ms is None:
        now_ms = current_time_ms()

    expired_tokens = []
    with manual_access_lock:
        for token, session in manual_access_sessions.items():
            if int(session.get("expires_ms", 0)) <= now_ms:
                expired_tokens.append(token)

        for token in expired_tokens:
            manual_access_sessions.pop(token, None)


def create_manual_access_session(client_ip: str) -> str:
    now_ms = current_time_ms()
    cleanup_manual_access_sessions(now_ms)
    token = secrets.token_urlsafe(32)
    with manual_access_lock:
        manual_access_sessions[token] = {
            "client_ip": client_ip,
            "created_ms": now_ms,
            "expires_ms": now_ms + MANUAL_ACCESS_SESSION_TTL_MS,
        }
    return token


def revoke_manual_access_session(token: str) -> None:
    if not token:
        return
    with manual_access_lock:
        manual_access_sessions.pop(token, None)


def manual_access_session_valid(token: str, client_ip: str) -> bool:
    if not token:
        return False

    now_ms = current_time_ms()
    cleanup_manual_access_sessions(now_ms)

    with manual_access_lock:
        session = manual_access_sessions.get(token)
        if session is None:
            return False
        if str(session.get("client_ip", "")) != client_ip:
            return False

        session["expires_ms"] = now_ms + MANUAL_ACCESS_SESSION_TTL_MS
        return True


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


def normalize_task_selection_id(raw_task: Any) -> str:
    task_id = str(raw_task or "").strip().upper()
    if task_id not in TASK_SELECTION_TASKS:
        raise ValueError(f"selectedTask must be one of {list(TASK_SELECTION_TASKS.keys())}")
    return task_id


def build_task_selection_catalog() -> List[Dict[str, Any]]:
    catalog = []
    for task_id, definition in TASK_SELECTION_TASKS.items():
        catalog.append(
            {
                "id": task_id,
                "label": definition["label"],
                "name_zh": definition["label"],
                "summary": definition["summary"],
                "description": definition["description"],
                "placeholder_entry_stage": definition["default_stage"],
                "supports_resume": bool(definition["supports_resume"]),
                "is_manual_test": bool(definition["is_manual_test"]),
                "obstacle_interrupt_enabled": bool(definition["obstacle_interrupt_enabled"]),
            }
        )
    return catalog


def build_task_selection_state(
    selected_task: Optional[str] = None,
    overrides: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    task_id = selected_task or DEFAULT_TASK_SELECTION_TASK
    if task_id not in TASK_SELECTION_TASKS:
        task_id = DEFAULT_TASK_SELECTION_TASK

    definition = TASK_SELECTION_TASKS[task_id]
    state = {
        "selectedTask": task_id,
        "selectedTaskLabel": definition["label"],
        "selectedTaskName": definition["label"],
        "taskDescription": definition["description"],
        "taskSummary": definition["summary"],
        "taskStatus": "IDLE",
        "currentStage": "NONE",
        "resumeAvailable": False,
        "savedStage": None,
        "isManualTest": bool(definition["is_manual_test"]),
        "isObstacleInterruptEnabled": bool(definition["obstacle_interrupt_enabled"]),
        "obstacleInterruptActive": False,
        "supportsResume": bool(definition["supports_resume"]),
        "placeholderEntryStage": definition["default_stage"],
        "backendConnected": False,
        "backendBinding": "placeholder_only",
        "statusSource": "MOCK_UI_ONLY",
        "lastAction": "INIT",
        "lastMessage": "Task selection placeholder is ready. No Jetson/Arduino task runtime is connected yet.",
    }  # type: Dict[str, Any]

    if overrides:
        state.update(overrides)

    state["selectedTask"] = task_id
    state["selectedTaskLabel"] = definition["label"]
    state["selectedTaskName"] = definition["label"]
    state["taskDescription"] = definition["description"]
    state["taskSummary"] = definition["summary"]
    state["isManualTest"] = bool(definition["is_manual_test"])
    state["isObstacleInterruptEnabled"] = bool(definition["obstacle_interrupt_enabled"])
    state["supportsResume"] = bool(definition["supports_resume"])
    state["placeholderEntryStage"] = definition["default_stage"]

    state["taskStatus"] = str(state.get("taskStatus", "IDLE") or "IDLE").strip().upper()
    current_stage = str(state.get("currentStage", "NONE") or "NONE").strip().upper()
    state["currentStage"] = current_stage if current_stage else "NONE"

    saved_stage = state.get("savedStage")
    if saved_stage is None:
        normalized_saved_stage = None
    else:
        normalized_saved_stage = str(saved_stage).strip().upper() or None
        if normalized_saved_stage == "NONE":
            normalized_saved_stage = None

    if not state["supportsResume"]:
        normalized_saved_stage = None
        state["resumeAvailable"] = False

    state["savedStage"] = normalized_saved_stage
    state["resumeAvailable"] = bool(state.get("resumeAvailable")) and bool(normalized_saved_stage)
    state["obstacleInterruptActive"] = bool(state.get("obstacleInterruptActive", False))
    state["backendConnected"] = bool(state.get("backendConnected", False))
    state["backendBinding"] = str(state.get("backendBinding", "placeholder_only") or "placeholder_only")
    state["statusSource"] = str(state.get("statusSource", "MOCK_UI_ONLY") or "MOCK_UI_ONLY")
    state["lastAction"] = str(state.get("lastAction", "INIT") or "INIT").strip().upper()
    state["lastMessage"] = str(state.get("lastMessage", "") or "")
    state["lastUpdatedMs"] = current_time_ms()
    return state


def get_task_selection_state() -> Dict[str, Any]:
    with task_selection_lock:
        if not task_selection_state:
            task_selection_state.update(build_task_selection_state())
        return dict(task_selection_state)


def patch_task_selection_state(patch: Dict[str, Any]) -> Dict[str, Any]:
    with task_selection_lock:
        base_state = dict(task_selection_state) if task_selection_state else build_task_selection_state()
        merged_state = dict(base_state)
        merged_state.update(patch)
        next_task = str(merged_state.get("selectedTask", DEFAULT_TASK_SELECTION_TASK) or DEFAULT_TASK_SELECTION_TASK)
        next_state = build_task_selection_state(next_task, merged_state)
        task_selection_state.clear()
        task_selection_state.update(next_state)
        return dict(task_selection_state)


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
        open_command = str(unlock_config.get("door_command", "OPEN_DOOR"))
        close_command = str(unlock_config.get("close_door_command", "CLOSE_DOOR"))
        if command == open_command:
            open_window_ms = int(unlock_config.get("door_auto_close_ms", MOCK_UNLOCK_OPEN_MS))
            mock_unlock_until_ms = current_time_ms() + max(0, open_window_ms)
        elif command == close_command:
            mock_unlock_until_ms = 0
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


def trigger_task_selection_select(
    task_raw: Any,
    client_ip: str,
    source: str = "dashboard",
) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    try:
        task_id = normalize_task_selection_id(task_raw)
    except ValueError as exc:
        return False, {"ok": False, "message": str(exc)}, HTTPStatus.BAD_REQUEST

    definition = TASK_SELECTION_TASKS[task_id]
    snapshot = patch_task_selection_state(
        {
            "selectedTask": task_id,
            "taskStatus": "IDLE",
            "currentStage": "NONE",
            "resumeAvailable": False,
            "savedStage": None,
            "obstacleInterruptActive": False,
            "lastAction": "SELECT",
            "lastClient": client_ip,
            "lastSource": source,
            "lastMessage": (
                f"{definition['label']} selected. This is a UI placeholder only; "
                "the real Jetson/Arduino task runtime is not wired yet."
            ),
        }
    )
    return (
        True,
        {
            "ok": True,
            "message": snapshot["lastMessage"],
            "state": snapshot,
        },
        HTTPStatus.OK,
    )


def trigger_task_selection_start(
    client_ip: str,
    source: str = "dashboard",
) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    snapshot = get_task_selection_state()
    task_id = str(snapshot.get("selectedTask", DEFAULT_TASK_SELECTION_TASK))
    definition = TASK_SELECTION_TASKS.get(task_id, TASK_SELECTION_TASKS[DEFAULT_TASK_SELECTION_TASK])

    placeholder_stage = str(definition["default_stage"])
    if bool(definition["is_manual_test"]):
        placeholder_stage = "MANUAL_TEST_ONLY"

    next_state = patch_task_selection_state(
        {
            "selectedTask": task_id,
            "taskStatus": "START_REQUESTED",
            "currentStage": placeholder_stage,
            "resumeAvailable": False,
            "savedStage": None,
            "obstacleInterruptActive": False,
            "lastAction": "START",
            "lastClient": client_ip,
            "lastSource": source,
            "lastMessage": (
                f"Start requested for {definition['label']}. Placeholder only; "
                "no real task state machine is attached to this API yet."
            ),
        }
    )
    return (
        True,
        {
            "ok": True,
            "message": next_state["lastMessage"],
            "state": next_state,
        },
        HTTPStatus.OK,
    )


def trigger_task_selection_stop(
    client_ip: str,
    source: str = "dashboard",
) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    snapshot = get_task_selection_state()
    task_id = str(snapshot.get("selectedTask", DEFAULT_TASK_SELECTION_TASK))
    definition = TASK_SELECTION_TASKS.get(task_id, TASK_SELECTION_TASKS[DEFAULT_TASK_SELECTION_TASK])
    current_stage = str(snapshot.get("currentStage", "NONE") or "NONE").strip().upper()
    can_resume = bool(definition["supports_resume"]) and current_stage not in {"", "NONE"}
    saved_stage = current_stage if can_resume else None

    next_state = patch_task_selection_state(
        {
            "selectedTask": task_id,
            "taskStatus": "STOPPED",
            "currentStage": "NONE",
            "resumeAvailable": can_resume,
            "savedStage": saved_stage,
            "obstacleInterruptActive": False,
            "lastAction": "STOP",
            "lastClient": client_ip,
            "lastSource": source,
            "lastMessage": (
                f"Stop requested for {definition['label']}. "
                + (
                    f"Saved placeholder stage: {saved_stage}."
                    if saved_stage
                    else "No resumable placeholder stage is available."
                )
            ),
        }
    )
    return (
        True,
        {
            "ok": True,
            "message": next_state["lastMessage"],
            "state": next_state,
        },
        HTTPStatus.OK,
    )


def trigger_task_selection_resume(
    client_ip: str,
    source: str = "dashboard",
) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    snapshot = get_task_selection_state()
    task_id = str(snapshot.get("selectedTask", DEFAULT_TASK_SELECTION_TASK))
    if not bool(snapshot.get("supportsResume")):
        return (
            False,
            {"ok": False, "message": "the selected task does not expose resume in the current placeholder design"},
            HTTPStatus.CONFLICT,
        )

    saved_stage = snapshot.get("savedStage")
    if not saved_stage or not bool(snapshot.get("resumeAvailable")):
        return (
            False,
            {"ok": False, "message": "no placeholder saved stage is available for resume"},
            HTTPStatus.CONFLICT,
        )

    definition = TASK_SELECTION_TASKS.get(task_id, TASK_SELECTION_TASKS[DEFAULT_TASK_SELECTION_TASK])
    next_state = patch_task_selection_state(
        {
            "selectedTask": task_id,
            "taskStatus": "RESUME_REQUESTED",
            "currentStage": saved_stage,
            "resumeAvailable": False,
            "savedStage": saved_stage,
            "obstacleInterruptActive": False,
            "lastAction": "RESUME",
            "lastClient": client_ip,
            "lastSource": source,
            "lastMessage": (
                f"Resume requested for {definition['label']} from placeholder stage {saved_stage}. "
                "This remains UI-only until the runtime is connected."
            ),
        }
    )
    return (
        True,
        {
            "ok": True,
            "message": next_state["lastMessage"],
            "state": next_state,
        },
        HTTPStatus.OK,
    )


def trigger_task_selection_mock_update(
    data: Dict[str, Any],
    client_ip: str,
    source: str = "dashboard",
) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    patch = {}  # type: Dict[str, Any]

    if "selectedTask" in data:
        try:
            patch["selectedTask"] = normalize_task_selection_id(data.get("selectedTask"))
        except ValueError as exc:
            return False, {"ok": False, "message": str(exc)}, HTTPStatus.BAD_REQUEST

    if "taskStatus" in data:
        task_status = str(data.get("taskStatus", "") or "").strip().upper()
        if not task_status:
            return False, {"ok": False, "message": "taskStatus cannot be empty"}, HTTPStatus.BAD_REQUEST
        patch["taskStatus"] = task_status

    if "currentStage" in data:
        patch["currentStage"] = str(data.get("currentStage", "NONE") or "NONE").strip().upper() or "NONE"

    if "savedStage" in data:
        raw_saved_stage = data.get("savedStage")
        if raw_saved_stage in (None, "", "NONE"):
            patch["savedStage"] = None
        else:
            patch["savedStage"] = str(raw_saved_stage).strip().upper()

    if "resumeAvailable" in data:
        patch["resumeAvailable"] = bool(data.get("resumeAvailable"))

    if "obstacleInterruptActive" in data:
        patch["obstacleInterruptActive"] = bool(data.get("obstacleInterruptActive"))

    if "lastMessage" in data:
        patch["lastMessage"] = str(data.get("lastMessage", "") or "")

    if not patch:
        return False, {"ok": False, "message": "no mock task-selection fields were provided"}, HTTPStatus.BAD_REQUEST

    patch["statusSource"] = "MOCK_UI_ONLY"
    patch["backendConnected"] = False
    patch["backendBinding"] = "placeholder_only"
    patch["lastAction"] = str(data.get("action", "MOCK_UPDATE") or "MOCK_UPDATE").strip().upper()
    patch["lastClient"] = client_ip
    patch["lastSource"] = source

    next_state = patch_task_selection_state(patch)
    return (
        True,
        {
            "ok": True,
            "message": next_state["lastMessage"] or "task selection placeholder updated",
            "state": next_state,
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
    ok, message = verify_unlock_password(password)
    if not ok:
        record_unlock_event(False, message, client_ip)
        patch_state({
            "unlock_status": "REJECTED",
            "unlock_message": message,
            "unlock_last_client": client_ip,
        })
        return False, {"ok": False, "message": message}, HTTPStatus.FORBIDDEN

    return trigger_door_command(
        command=str(unlock_config.get("door_command", "OPEN_DOOR")),
        client_ip=client_ip,
        success_status="ACCEPTED",
        success_state_message="fixed password accepted, unlock command sent",
        success_response_message="unlock command sent",
        apply_cooldown=True,
    )


def trigger_close_door(client_ip: str) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    return trigger_door_command(
        command=str(unlock_config.get("close_door_command", "CLOSE_DOOR")),
        client_ip=client_ip,
        success_status="CLOSE_SENT",
        success_state_message="close-door command sent",
        success_response_message="close-door command sent",
        apply_cooldown=False,
    )


def trigger_door_command(
    command: str,
    client_ip: str,
    *,
    success_status: str,
    success_state_message: str,
    success_response_message: str,
    apply_cooldown: bool,
) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    global last_unlock_sent_ms

    cooldown_ms = int(unlock_config.get("unlock_cooldown_ms", 0))
    door_auto_close_ms = int(unlock_config.get("door_auto_close_ms", MOCK_UNLOCK_OPEN_MS))
    now_ms = current_time_ms()
    event_success = False
    event_reason = ""
    event_command = ""
    state_patch = {}  # type: Dict[str, Any]
    response_payload = {}  # type: Dict[str, Any]
    response_status = HTTPStatus.OK

    with unlock_lock:
        if apply_cooldown and cooldown_ms > 0 and now_ms - last_unlock_sent_ms < cooldown_ms:
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
                if apply_cooldown:
                    last_unlock_sent_ms = now_ms
                event_success = True
                event_reason = "command queued"
                event_command = command
                state_patch = {
                    "unlock_status": success_status,
                    "unlock_message": success_state_message,
                    "unlock_last_client": client_ip,
                    "unlock_last_command": command,
                    "door_auto_close_ms": door_auto_close_ms,
                    "cargo_door": "OPEN" if apply_cooldown else "CLOSED",
                }
                if apply_cooldown:
                    state_patch["unlock_last_ts_ms"] = now_ms
                response_payload = {
                    "ok": True,
                    "message": success_response_message,
                    "command": command,
                    "door_auto_close_ms": door_auto_close_ms,
                }
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

    def cookie_value(self, name: str) -> str:
        raw_cookie = self.headers.get("Cookie", "")
        if not raw_cookie:
            return ""

        cookie = SimpleCookie()
        try:
            cookie.load(raw_cookie)
        except Exception:
            return ""

        morsel = cookie.get(name)
        if morsel is None:
            return ""
        return morsel.value

    def manual_access_allowed(self, client_ip: str) -> bool:
        if is_loopback_client(client_ip):
            return True

        session_token = self.cookie_value(MANUAL_ACCESS_COOKIE_NAME)
        return manual_access_session_valid(session_token, client_ip)

    def require_manual_access(self, client_ip: str, capability: str) -> bool:
        if self.manual_access_allowed(client_ip):
            return True

        self.send_json(
            {
                "ok": False,
                "message": (
                    f"{capability} requires loopback access or a successful LAN manual-access login "
                    "from /manual"
                ),
            },
            HTTPStatus.FORBIDDEN,
        )
        return False

    def manual_access_cookie_header(self, token: str, *, expire_now: bool = False) -> str:
        max_age = 0 if expire_now else MANUAL_ACCESS_SESSION_TTL_MS // 1000
        value = "" if expire_now else token
        return (
            f"{MANUAL_ACCESS_COOKIE_NAME}={value}; Path=/; HttpOnly; SameSite=Strict; Max-Age={max_age}"
        )

    def do_GET(self) -> None:
        parsed = urlparse(self.path)

        if parsed.path == "/":
            self.serve_file(HOME_HTML)
            return

        if parsed.path == "/home":
            self.serve_file(HOME_HTML)
            return

        if parsed.path == "/dashboard":
            self.serve_file(DASHBOARD_HTML)
            return

        if parsed.path == "/telemetry":
            self.serve_file(DASHBOARD_HTML)
            return

        if parsed.path == "/manual":
            client_ip = self.client_address[0] if self.client_address else "unknown"
            self.serve_file(MANUAL_HTML if self.manual_access_allowed(client_ip) else MANUAL_ACCESS_HTML)
            return

        if parsed.path == "/tasks":
            self.serve_file(TASKS_HTML)
            return

        if parsed.path == "/status":
            self.serve_file(STATUS_HTML)
            return

        if parsed.path == "/debug":
            self.serve_file(DEBUG_HTML)
            return

        if parsed.path == "/unlock":
            self.serve_file(UNLOCK_HTML)
            return

        if parsed.path == "/arm":
            self.redirect("/manual")
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

        if parsed.path == "/api/task-selection/state":
            self.serve_task_selection_state()
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

        if parsed.path == "/api/manual-access/login":
            self.handle_manual_access_login_post()
            return

        if parsed.path == "/api/manual-access/logout":
            self.handle_manual_access_logout_post()
            return

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

        if parsed.path == "/api/task-selection/select":
            self.handle_task_selection_select_post()
            return

        if parsed.path == "/api/task-selection/start":
            self.handle_task_selection_start_post()
            return

        if parsed.path == "/api/task-selection/stop":
            self.handle_task_selection_stop_post()
            return

        if parsed.path == "/api/task-selection/resume":
            self.handle_task_selection_resume_post()
            return

        if parsed.path == "/api/task-selection/mock-update":
            self.handle_task_selection_mock_update_post()
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def send_json(
        self,
        payload: Any,
        status: HTTPStatus = HTTPStatus.OK,
        extra_headers: Optional[List[Tuple[str, str]]] = None,
    ) -> None:
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        if extra_headers:
            for name, value in extra_headers:
                self.send_header(name, value)
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def redirect(self, location: str, status: HTTPStatus = HTTPStatus.FOUND) -> None:
        body = (
            "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
            f"<meta http-equiv=\"refresh\" content=\"0; url={location}\">"
            f"<title>Redirect</title></head><body>Redirecting to <a href=\"{location}\">{location}</a>.</body></html>"
        ).encode("utf-8")
        self.send_response(status)
        self.send_header("Location", location)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

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

    def serve_task_selection_state(self) -> None:
        self.send_json(get_task_selection_state())

    def serve_public_config(self) -> None:
        payload = {
            "robot_id": unlock_config.get("robot_id", "amr01"),
            "unlock_mode": unlock_config.get("unlock_mode", "fixed"),
            "http_paths": {
                "home": "/",
                "dashboard": "/dashboard",
                "unlock": "/unlock",
                "telemetry": "/telemetry",
                "manual": "/manual",
                "tasks": "/tasks",
                "arm": "/arm",
                "status": "/status",
                "debug": "/debug",
            },
            "manual_control": {
                "api_path": "/api/control",
                "directions": sorted(ALLOWED_DRIVE_DIRECTIONS),
                "default_power": DEFAULT_DRIVE_POWER,
                "max_power": MAX_DRIVE_POWER,
                "loopback_only": False,
                "requires_lan_password": True,
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
                "loopback_only": False,
                "requires_lan_password": True,
            },
            "unlock_control": {
                "api_path": "/api/unlock",
                "door_command": unlock_config.get("door_command", "OPEN_DOOR"),
                "close_door_command": unlock_config.get("close_door_command", "CLOSE_DOOR"),
                "door_auto_close_ms": int(unlock_config.get("door_auto_close_ms", MOCK_UNLOCK_OPEN_MS)),
                "supports_manual_close": True,
            },
            "mode_control": {
                "api_path": "/api/mode",
                "actions": sorted(ALLOWED_MODE_ACTIONS),
                "protocol_version": MODE_PROTOCOL_VERSION,
                "loopback_only": False,
                "requires_lan_password": True,
            },
            "task_control": {
                "api_path": "/api/task",
                "actions": sorted(ALLOWED_TASK_ACTIONS),
                "loopback_only": False,
                "requires_lan_password": True,
            },
            "manual_access": {
                "login_api_path": "/api/manual-access/login",
                "logout_api_path": "/api/manual-access/logout",
                "loopback_bypasses_password": True,
                "password_required_for_lan": True,
                "session_ttl_ms": MANUAL_ACCESS_SESSION_TTL_MS,
            },
            "task_selection": {
                "backend_binding": "placeholder_only",
                "api_paths": {
                    "state": "/api/task-selection/state",
                    "select": "/api/task-selection/select",
                    "start": "/api/task-selection/start",
                    "stop": "/api/task-selection/stop",
                    "resume": "/api/task-selection/resume",
                    "mock_update": "/api/task-selection/mock-update",
                },
                "tasks": build_task_selection_catalog(),
                "status_values": list(TASK_SELECTION_STATUS_VALUES),
                "state_fields": list(TASK_SELECTION_STATE_FIELDS),
                "loopback_only_for_post": True,
                "notes": [
                    "This is a UI/task-interface placeholder only.",
                    "No real Jetson/Arduino task runtime is attached yet.",
                    "mock_update is reserved for future runtime-to-UI state sync.",
                ],
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

    def handle_manual_access_login_post(self) -> None:
        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        client_ip = self.client_address[0] if self.client_address else "unknown"
        password = str(data.get("password", ""))
        if password != MANUAL_ACCESS_PASSWORD:
            self.send_json(
                {"ok": False, "message": "wrong manual-control password"},
                HTTPStatus.FORBIDDEN,
            )
            return

        session_token = create_manual_access_session(client_ip)
        self.send_json(
            {
                "ok": True,
                "message": "manual access granted",
                "redirect": "/manual",
            },
            HTTPStatus.OK,
            extra_headers=[("Set-Cookie", self.manual_access_cookie_header(session_token))],
        )

    def handle_manual_access_logout_post(self) -> None:
        session_token = self.cookie_value(MANUAL_ACCESS_COOKIE_NAME)
        revoke_manual_access_session(session_token)
        self.send_json(
            {"ok": True, "message": "manual access cleared"},
            HTTPStatus.OK,
            extra_headers=[("Set-Cookie", self.manual_access_cookie_header("", expire_now=True))],
        )

    def handle_unlock_post(self) -> None:
        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        client_ip = self.client_address[0] if self.client_address else "unknown"
        action = str(data.get("action", "OPEN")).upper()

        if action == "OPEN":
            password = str(data.get("password", ""))
            _, payload, status = trigger_unlock(password, client_ip)
            self.send_json(payload, status)
            return

        if action == "CLOSE":
            _, payload, status = trigger_close_door(client_ip)
            self.send_json(payload, status)
            return

        self.send_json({"ok": False, "message": f"unsupported unlock action: {action}"}, HTTPStatus.BAD_REQUEST)

    def handle_control_post(self) -> None:
        client_ip = self.client_address[0] if self.client_address else "unknown"
        if not self.require_manual_access(client_ip, "manual drive"):
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
        if not self.require_manual_access(client_ip, "arm control"):
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
        if not self.require_manual_access(client_ip, "mode control"):
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
        if not self.require_manual_access(client_ip, "task control"):
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

    def handle_task_selection_select_post(self) -> None:
        client_ip = self.client_address[0] if self.client_address else "unknown"
        if not is_loopback_client(client_ip):
            self.send_json(
                {"ok": False, "message": "task selection is only allowed from loopback clients"},
                HTTPStatus.FORBIDDEN,
            )
            return

        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        selected_task = data.get("selectedTask", "")
        source = str(data.get("source", "dashboard"))
        ok, payload, status = trigger_task_selection_select(selected_task, client_ip, source=source)
        self.send_json(payload, status)

    def handle_task_selection_start_post(self) -> None:
        client_ip = self.client_address[0] if self.client_address else "unknown"
        if not is_loopback_client(client_ip):
            self.send_json(
                {"ok": False, "message": "task selection start is only allowed from loopback clients"},
                HTTPStatus.FORBIDDEN,
            )
            return

        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        source = str(data.get("source", "dashboard"))
        ok, payload, status = trigger_task_selection_start(client_ip, source=source)
        self.send_json(payload, status)

    def handle_task_selection_stop_post(self) -> None:
        client_ip = self.client_address[0] if self.client_address else "unknown"
        if not is_loopback_client(client_ip):
            self.send_json(
                {"ok": False, "message": "task selection stop is only allowed from loopback clients"},
                HTTPStatus.FORBIDDEN,
            )
            return

        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        source = str(data.get("source", "dashboard"))
        ok, payload, status = trigger_task_selection_stop(client_ip, source=source)
        self.send_json(payload, status)

    def handle_task_selection_resume_post(self) -> None:
        client_ip = self.client_address[0] if self.client_address else "unknown"
        if not is_loopback_client(client_ip):
            self.send_json(
                {"ok": False, "message": "task selection resume is only allowed from loopback clients"},
                HTTPStatus.FORBIDDEN,
            )
            return

        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        source = str(data.get("source", "dashboard"))
        ok, payload, status = trigger_task_selection_resume(client_ip, source=source)
        self.send_json(payload, status)

    def handle_task_selection_mock_update_post(self) -> None:
        client_ip = self.client_address[0] if self.client_address else "unknown"
        if not is_loopback_client(client_ip):
            self.send_json(
                {"ok": False, "message": "task selection mock update is only allowed from loopback clients"},
                HTTPStatus.FORBIDDEN,
            )
            return

        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        source = str(data.get("source", "dashboard"))
        ok, payload, status = trigger_task_selection_mock_update(data, client_ip, source=source)
        self.send_json(payload, status)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Jetson Nano serial telemetry dashboard")
    parser.add_argument("--serial", default="/dev/ttyUSB0", help="Arduino serial device path")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind host")
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
        "door_auto_close_ms": int(unlock_config.get("door_auto_close_ms", MOCK_UNLOCK_OPEN_MS)),
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
    patch_task_selection_state({})

    worker = threading.Thread(
        target=mock_reader_loop if args.mock else serial_reader_loop,
        args=() if args.mock else (args.serial, args.baud),
        daemon=True,
    )
    worker.start()

    server = ThreadedHTTPServer((args.host, args.http_port), DashboardHandler)
    print(f"[http] home ready at http://{args.host}:{args.http_port}/")
    print(f"[http] dashboard ready at http://{args.host}:{args.http_port}/dashboard")
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

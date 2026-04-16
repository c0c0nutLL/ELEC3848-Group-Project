import argparse
import json
import queue
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from socketserver import ThreadingMixIn
from typing import Any, Deque, Dict, List, Tuple
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
DEFAULT_DRIVE_POWER = 96
MAX_DRIVE_POWER = 255
MOCK_UNLOCK_OPEN_MS = 5000
MOCK_DISTANCE_MIN_CM = 8.0
MOCK_DISTANCE_MAX_CM = 150.0

state_lock = threading.Lock()
clients_lock = threading.Lock()
unlock_lock = threading.Lock()
control_lock = threading.Lock()

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


def queue_serial_command(command: str) -> bool:
    global mock_unlock_until_ms

    if bridge_mode == "mock":
        print(f"[serial-mock] command={command}")
        if command == str(unlock_config.get("door_command", "OPEN_DOOR")):
            mock_unlock_until_ms = current_time_ms() + MOCK_UNLOCK_OPEN_MS
        return True

    try:
        serial_command_queue.put_nowait(command)
        return True
    except queue.Full:
        return False


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
        f"result={packet.get('delivery_result', '?')} "
        f"direction={packet.get('control_direction', '?')}"
    )


def trigger_manual_drive(
    direction_raw: Any,
    power_raw: Any,
    client_ip: str,
    source: str = "dashboard",
) -> Tuple[bool, Dict[str, Any], HTTPStatus]:
    try:
        direction = normalize_direction(direction_raw)
        power = normalize_power(power_raw)
    except ValueError as exc:
        return False, {"ok": False, "message": str(exc)}, HTTPStatus.BAD_REQUEST

    if direction != "STOP" and power == 0:
        direction = "STOP"

    command = f"DRIVE {direction} {power}"
    if not queue_serial_command(command):
        return False, {"ok": False, "message": "command queue is full"}, HTTPStatus.SERVICE_UNAVAILABLE

    status = "ACTIVE" if direction != "STOP" else "IDLE"
    message = "stop command queued" if direction == "STOP" else f"{direction.lower()} command queued"

    patch_state(
        {
            **set_control_state(direction, power, status, source),
            "state": "IDLE" if direction == "STOP" else f"MANUAL_{direction}",
            "delivery_result": "READY" if direction == "STOP" else "MANUAL_CONTROL",
            "target_station": "MANUAL",
            "control_message": message,
            "control_last_client": client_ip,
            "fault": "",
        },
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
            update_state({
                "state": "BRIDGE_WAITING",
                "delivery_result": "SERIAL_DISCONNECTED",
                "bridge_mode": "serial",
                "bridge_status": "DISCONNECTED",
                "control_direction": "STOP",
                "control_status": "OFFLINE",
                "right_cm": None,
                "left_cm": None,
                "avg_cm": None,
                "diff_cm": None,
                "heading_deg": None,
                "fault": str(exc),
            })
            time.sleep(2)


def mock_reader_loop() -> None:
    start_ms = current_time_ms()
    while not stop_event.is_set():
        with control_lock:
            direction = str(control_state["direction"])
            power = int(control_state["power"])
            control_status = str(control_state["status"])
            control_source = str(control_state["source"])
            control_last_command_ms = int(control_state["last_command_ms"])

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
        state = "IDLE" if direction == "STOP" else f"MANUAL_{direction}"
        delivery_result = "READY" if direction == "STOP" else "MANUAL_CONTROL"
        if direction == "STOP" and cargo_door == "OPEN":
            state = "REMOTE_UNLOCK"
            delivery_result = "UNLOCKED"

        packet = {
            "ts_ms": current_time_ms() - start_ms,
            "state": state,
            "target_station": "MANUAL",
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
            "control_direction": direction,
            "motor_pwm": power,
            "control_status": control_status,
            "control_source": control_source,
            "control_last_command_ms": control_last_command_ms or None,
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

    with unlock_lock:
        if cooldown_ms > 0 and now_ms - last_unlock_sent_ms < cooldown_ms:
            retry_ms = cooldown_ms - (now_ms - last_unlock_sent_ms)
            message = f"cooldown active, retry in {retry_ms} ms"
            record_unlock_event(False, message, client_ip)
            patch_state({
                "unlock_status": "COOLDOWN",
                "unlock_message": message,
                "unlock_last_client": client_ip,
            })
            return False, {"ok": False, "message": message}, HTTPStatus.TOO_MANY_REQUESTS

        command = str(unlock_config.get("door_command", "OPEN_DOOR"))
        queued = queue_serial_command(command)
        if not queued:
            message = "command queue is full"
            record_unlock_event(False, message, client_ip)
            patch_state({
                "unlock_status": "QUEUE_FULL",
                "unlock_message": message,
                "unlock_last_client": client_ip,
            })
            return False, {"ok": False, "message": message}, HTTPStatus.SERVICE_UNAVAILABLE

        last_unlock_sent_ms = now_ms

    record_unlock_event(True, "command queued", client_ip, command_sent=command)
    patch_state({
        "unlock_status": "ACCEPTED",
        "unlock_message": "fixed password accepted, unlock command sent",
        "unlock_last_client": client_ip,
        "unlock_last_command": command,
        "unlock_last_ts_ms": now_ms,
    })
    return True, {"ok": True, "message": "unlock command sent", "command": command}, HTTPStatus.OK


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
        try:
            data = self.read_json_body()
        except Exception as exc:
            self.send_json({"ok": False, "message": f"invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        client_ip = self.client_address[0] if self.client_address else "unknown"
        direction = data.get("direction", "STOP")
        power = data.get("power", DEFAULT_DRIVE_POWER)
        source = str(data.get("source", "dashboard"))
        ok, payload, status = trigger_manual_drive(direction, power, client_ip, source=source)
        self.send_json(payload, status)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Jetson Nano serial telemetry dashboard")
    parser.add_argument("--serial", default="/dev/ttyACM0", help="Arduino serial device path")
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

    patch_state({
        "bridge_mode": bridge_mode,
        "bridge_status": "STARTING",
        "robot_id": unlock_config.get("robot_id", "amr01"),
        "unlock_mode": unlock_config.get("unlock_mode", "fixed"),
        "unlock_status": "IDLE",
        "unlock_message": "",
        **build_control_patch(),
    })

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

from __future__ import annotations

import argparse
import json
import queue
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

try:
    import serial  # type: ignore
except ImportError:
    serial = None


BASE_DIR = Path(__file__).resolve().parent
DASHBOARD_HTML = BASE_DIR / "dashboard.html"
UNLOCK_HTML = BASE_DIR / "unlock.html"
UNLOCK_CONFIG_PATH = BASE_DIR / "unlock_config.json"

DEFAULT_UNLOCK_CONFIG: dict[str, Any] = {
    "robot_id": "amr01",
    "unlock_mode": "fixed",
    "fixed_password": "3848",
    "door_command": "OPEN_DOOR",
    "unlock_cooldown_ms": 5000,
}

state_lock = threading.Lock()
clients_lock = threading.Lock()
unlock_lock = threading.Lock()

latest_state: dict[str, Any] = {}
state_history: deque[dict[str, Any]] = deque(maxlen=100)
unlock_history: deque[dict[str, Any]] = deque(maxlen=50)
client_queues: list[queue.Queue[str]] = []
serial_command_queue: queue.Queue[str] = queue.Queue(maxsize=20)
stop_event = threading.Event()

unlock_config: dict[str, Any] = dict(DEFAULT_UNLOCK_CONFIG)
bridge_mode = "serial"
last_unlock_sent_ms = 0


def current_time_ms() -> int:
    return int(time.time() * 1000)


def load_unlock_config(config_path: Path) -> dict[str, Any]:
    config = dict(DEFAULT_UNLOCK_CONFIG)
    if config_path.exists():
        loaded = json.loads(config_path.read_text(encoding="utf-8"))
        if isinstance(loaded, dict):
            config.update(loaded)
    return config


def broadcast_state(packet: dict[str, Any]) -> None:
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


def update_state(packet: dict[str, Any]) -> None:
    packet["jetson_received_ms"] = current_time_ms()

    with state_lock:
        latest_state.clear()
        latest_state.update(packet)
        state_history.appendleft(dict(packet))

    broadcast_state(packet)


def patch_state(patch: dict[str, Any], add_history: bool = False) -> None:
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
    if bridge_mode == "mock":
        print(f"[serial-mock] command={command}")
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
    print(f"[serial] state={packet.get('state', '?')} result={packet.get('delivery_result', '?')}")


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
                "fault": str(exc),
            })
            time.sleep(2)


def mock_reader_loop() -> None:
    demo_states = [
        {"state": "IDLE", "delivery_result": "READY"},
        {"state": "MOVING", "delivery_result": "RUNNING"},
        {"state": "DOCKING", "delivery_result": "RUNNING"},
        {"state": "RFID_CHECK", "delivery_result": "VERIFYING"},
        {"state": "CARGO_RELEASE", "delivery_result": "DELIVERING"},
        {"state": "REPORTING", "delivery_result": "UPLOADING"},
        {"state": "DONE", "delivery_result": "SUCCESS"},
    ]

    index = 0
    start_ms = current_time_ms()
    while not stop_event.is_set():
        state = dict(demo_states[index % len(demo_states)])
        phase = index % len(demo_states)
        packet = {
            "ts_ms": current_time_ms() - start_ms,
            "target_station": "A1",
            "right_cm": round(60.0 - phase * 7.0, 1),
            "left_cm": round(58.0 - phase * 7.0, 1),
            "avg_cm": round(59.0 - phase * 7.0, 1),
            "diff_cm": 2.0,
            "heading_deg": round(max(0.0, 3.5 - phase * 0.4), 1),
            "rfid_tag": "STATION_A1" if phase >= 3 else "",
            "rfid_match": phase >= 3,
            "cargo_door": "OPEN" if phase == 4 else "CLOSED",
            "fault": "",
            "bridge_mode": "mock",
            "bridge_status": "CONNECTED",
            **state,
        }
        update_state(packet)
        index += 1
        time.sleep(1.0)


def verify_unlock_password(password: str) -> tuple[bool, str]:
    mode = str(unlock_config.get("unlock_mode", "fixed"))
    if mode != "fixed":
        return False, f"unlock mode {mode} is not supported yet"

    expected = str(unlock_config.get("fixed_password", ""))
    if password != expected:
        return False, "wrong password"

    return True, "password accepted"


def trigger_unlock(password: str, client_ip: str) -> tuple[bool, dict[str, Any], HTTPStatus]:
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
        }
        self.send_json(payload)

    def serve_events(self) -> None:
        client_queue: queue.Queue[str] = queue.Queue(maxsize=20)
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

    def read_json_body(self) -> dict[str, Any]:
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
    })

    worker = threading.Thread(
        target=mock_reader_loop if args.mock else serial_reader_loop,
        args=() if args.mock else (args.serial, args.baud),
        daemon=True,
    )
    worker.start()

    server = ThreadingHTTPServer((args.host, args.http_port), DashboardHandler)
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

import argparse
import sys
import threading
import time
from http import HTTPStatus
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse

import server as dashboard_server


BASE_DIR = Path(__file__).resolve().parent
YOLO_TESTS_DIR = BASE_DIR.parent / "jetson_yolo_tests"

if str(YOLO_TESTS_DIR) not in sys.path:
    sys.path.insert(0, str(YOLO_TESTS_DIR))

_yolo_camera_import_error = ""
try:
    import camera_detection_test as yolo_camera_test  # type: ignore
except Exception as exc:  # pragma: no cover - import path failure is reported through API state
    yolo_camera_test = None
    _yolo_camera_import_error = str(exc)


yolo_state_lock = threading.Lock()
yolo_state = {}  # type: Dict[str, Any]


def current_time_ms() -> int:
    return dashboard_server.current_time_ms()


def build_initial_yolo_state(args: argparse.Namespace) -> Dict[str, Any]:
    enabled = not bool(args.disable_yolo)
    status = "DISABLED" if not enabled else "STARTING"
    return {
        "yolo_enabled": enabled,
        "yolo_status": status,
        "runtime_ready": False,
        "model_loaded": False,
        "camera_ok": False,
        "camera_source": str(args.yolo_camera),
        "model_path": str(args.yolo_model),
        "device": str(args.yolo_device or ""),
        "conf_threshold": float(args.yolo_conf),
        "imgsz": int(args.yolo_imgsz),
        "detected": False,
        "person_detected": False,
        "classes": [],
        "class_counts": {},
        "count": 0,
        "top_class": "",
        "top_score": None,
        "person_boxes": [],
        "person_box_count": 0,
        "person_box": None,
        "person_center": None,
        "last_update_ms": None,
        "error_message": "",
    }


def build_yolo_state_patch(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "yolo_enabled": bool(snapshot.get("yolo_enabled", False)),
        "yolo_status": str(snapshot.get("yolo_status", "")),
        "yolo_runtime_ready": bool(snapshot.get("runtime_ready", False)),
        "yolo_model_loaded": bool(snapshot.get("model_loaded", False)),
        "yolo_camera_ok": bool(snapshot.get("camera_ok", False)),
        "yolo_camera_source": snapshot.get("camera_source"),
        "yolo_model_path": snapshot.get("model_path"),
        "yolo_device": snapshot.get("device"),
        "yolo_detected": bool(snapshot.get("detected", False)),
        "yolo_person_detected": bool(snapshot.get("person_detected", False)),
        "yolo_classes": list(snapshot.get("classes", [])),
        "yolo_class_counts": dict(snapshot.get("class_counts", {})),
        "yolo_count": int(snapshot.get("count", 0) or 0),
        "yolo_top_class": str(snapshot.get("top_class", "") or ""),
        "yolo_top_score": snapshot.get("top_score"),
        "yolo_person_boxes": list(snapshot.get("person_boxes", [])),
        "yolo_person_box_count": int(snapshot.get("person_box_count", 0) or 0),
        "yolo_person_box": snapshot.get("person_box"),
        "yolo_person_center": snapshot.get("person_center"),
        "yolo_last_update_ms": snapshot.get("last_update_ms"),
        "yolo_error_message": str(snapshot.get("error_message", "") or ""),
    }


def get_yolo_state_snapshot() -> Dict[str, Any]:
    with yolo_state_lock:
        return dict(yolo_state)


def replace_yolo_state(next_state: Dict[str, Any]) -> Dict[str, Any]:
    snapshot = dict(next_state)
    with yolo_state_lock:
        yolo_state.clear()
        yolo_state.update(snapshot)
        current_snapshot = dict(yolo_state)

    dashboard_server.patch_state(build_yolo_state_patch(current_snapshot))
    return current_snapshot


def patch_yolo_state(patch: Dict[str, Any]) -> Dict[str, Any]:
    with yolo_state_lock:
        yolo_state.update(patch)
        snapshot = dict(yolo_state)

    dashboard_server.patch_state(build_yolo_state_patch(snapshot))
    return snapshot


def import_yolo_runtime() -> Any:
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise RuntimeError("Missing dependency: cv2") from exc

    try:
        from ultralytics import YOLO  # type: ignore
    except ImportError as exc:
        raise RuntimeError("Missing dependency: ultralytics") from exc

    return cv2, YOLO


def ordered_unique(values: List[str]) -> List[str]:
    seen = set()
    ordered = []
    for value in values:
        if value in seen:
            continue
        seen.add(value)
        ordered.append(value)
    return ordered


def tensor_to_list(value: Any) -> List[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "tolist"):
        converted = value.tolist()
        if isinstance(converted, list):
            return converted
        return [converted]
    try:
        return list(value)
    except TypeError:
        return [value]


def class_name_from_index(names: Any, index: int) -> str:
    if isinstance(names, dict):
        if index in names:
            return str(names[index])
        key = str(index)
        if key in names:
            return str(names[key])
    elif isinstance(names, list) and 0 <= index < len(names):
        return str(names[index])
    return str(index)


def extract_detection_state(result: Any, model: Any) -> Dict[str, Any]:
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return {
            "detected": False,
            "person_detected": False,
            "classes": [],
            "class_counts": {},
            "count": 0,
            "top_class": "",
            "top_score": None,
            "person_boxes": [],
            "person_box_count": 0,
            "person_box": None,
            "person_center": None,
        }

    class_ids = tensor_to_list(getattr(boxes, "cls", None))
    confidences = tensor_to_list(getattr(boxes, "conf", None))
    xyxy_rows = tensor_to_list(getattr(boxes, "xyxy", None))
    names = getattr(result, "names", None)
    if not names:
        names = getattr(model, "names", {})

    detections = []  # type: List[str]
    class_counts = {}  # type: Dict[str, int]
    person_boxes = []  # type: List[Dict[str, Any]]
    primary_person_box = None  # type: Optional[Dict[str, Any]]
    top_class = ""
    top_score = None  # type: Optional[float]

    for index, raw_class_id in enumerate(class_ids):
        try:
            class_id = int(raw_class_id)
        except (TypeError, ValueError):
            continue

        class_name = class_name_from_index(names, class_id)
        detections.append(class_name)
        class_counts[class_name] = class_counts.get(class_name, 0) + 1

        score = None  # type: Optional[float]
        if index < len(confidences):
            try:
                score = float(confidences[index])
            except (TypeError, ValueError):
                score = None

        if score is not None and (top_score is None or score > top_score):
            top_score = score
            top_class = class_name

        if str(class_name).lower() == "person":
            xyxy = xyxy_rows[index] if index < len(xyxy_rows) else None
            if isinstance(xyxy, (list, tuple)) and len(xyxy) >= 4:
                try:
                    x1 = float(xyxy[0])
                    y1 = float(xyxy[1])
                    x2 = float(xyxy[2])
                    y2 = float(xyxy[3])
                except (TypeError, ValueError):
                    x1 = y1 = x2 = y2 = None  # type: ignore[assignment]

                if x1 is not None:
                    center_x = (x1 + x2) / 2.0
                    center_y = (y1 + y2) / 2.0
                    person_box = {
                        "x1": round(x1, 2),
                        "y1": round(y1, 2),
                        "x2": round(x2, 2),
                        "y2": round(y2, 2),
                        "center_x": round(center_x, 2),
                        "center_y": round(center_y, 2),
                        "confidence": round(score, 4) if score is not None else None,
                    }
                    person_boxes.append(person_box)
                    if primary_person_box is None:
                        primary_person_box = person_box
                    elif score is not None:
                        primary_score = primary_person_box.get("confidence")
                        if primary_score is None or score > float(primary_score):
                            primary_person_box = person_box

    unique_classes = ordered_unique(detections)
    person_detected = any(str(name).lower() == "person" for name in detections)
    person_center = None  # type: Optional[Dict[str, float]]
    if primary_person_box is not None:
        person_center = {
            "x": float(primary_person_box["center_x"]),
            "y": float(primary_person_box["center_y"]),
        }

    return {
        "detected": len(detections) > 0,
        "person_detected": person_detected,
        "classes": unique_classes,
        "class_counts": class_counts,
        "count": len(detections),
        "top_class": top_class,
        "top_score": round(top_score, 4) if top_score is not None else None,
        "person_boxes": person_boxes,
        "person_box_count": len(person_boxes),
        "person_box": primary_person_box,
        "person_center": person_center,
    }


def reset_detection_fields() -> Dict[str, Any]:
    return {
        "detected": False,
        "person_detected": False,
        "classes": [],
        "class_counts": {},
        "count": 0,
        "top_class": "",
        "top_score": None,
        "person_boxes": [],
        "person_box_count": 0,
        "person_box": None,
        "person_center": None,
        "last_update_ms": None,
    }


def set_yolo_error(status: str, message: str, extra_patch: Optional[Dict[str, Any]] = None) -> None:
    patch = {
        "yolo_status": status,
        "camera_ok": False,
        "error_message": str(message),
    }
    patch.update(reset_detection_fields())
    if extra_patch:
        patch.update(extra_patch)
    patch_yolo_state(patch)


def yolo_worker_loop(args: argparse.Namespace) -> None:
    if args.disable_yolo:
        replace_yolo_state(build_initial_yolo_state(args))
        return

    if yolo_camera_test is None:
        set_yolo_error(
            "IMPORT_ERROR",
            "Failed to import camera_detection_test.py: {}".format(_yolo_camera_import_error or "unknown error"),
            {"runtime_ready": False, "model_loaded": False},
        )
        return

    try:
        cv2, YOLO = import_yolo_runtime()
    except Exception as exc:
        set_yolo_error(
            "RUNTIME_ERROR",
            "{}. Use the Jetson Docker YOLO runtime or another environment with OpenCV + ultralytics.".format(exc),
            {"runtime_ready": False, "model_loaded": False},
        )
        return

    patch_yolo_state(
        {
            "runtime_ready": True,
            "yolo_status": "LOADING_MODEL",
            "error_message": "",
        }
    )

    try:
        model = YOLO(args.yolo_model)
    except Exception as exc:
        set_yolo_error("MODEL_ERROR", "Failed to load YOLO model: {}".format(exc), {"runtime_ready": True, "model_loaded": False})
        return

    patch_yolo_state(
        {
            "runtime_ready": True,
            "model_loaded": True,
            "yolo_status": "WAITING_CAMERA",
            "error_message": "",
        }
    )

    predict_kwargs = {
        "conf": args.yolo_conf,
        "imgsz": args.yolo_imgsz,
        "verbose": False,
    }
    if args.yolo_device:
        predict_kwargs["device"] = args.yolo_device

    retry_seconds = max(0.2, float(args.yolo_retry_ms) / 1000.0)

    while not dashboard_server.stop_event.is_set():
        try:
            capture = yolo_camera_test.open_camera(
                cv2,
                args.yolo_camera,
                args.yolo_width,
                args.yolo_height,
                args.yolo_fps,
                args.yolo_flip_method,
            )
        except ValueError as exc:
            set_yolo_error("CAMERA_CONFIG_ERROR", str(exc), {"runtime_ready": True, "model_loaded": True})
            return
        except Exception as exc:
            set_yolo_error("CAMERA_OPEN_ERROR", "Failed to create camera pipeline: {}".format(exc), {"runtime_ready": True, "model_loaded": True})
            time.sleep(retry_seconds)
            continue

        if not capture.isOpened():
            try:
                capture.release()
            except Exception:
                pass
            set_yolo_error(
                "CAMERA_OPEN_ERROR",
                "Failed to open camera input: {}".format(args.yolo_camera),
                {"runtime_ready": True, "model_loaded": True},
            )
            time.sleep(retry_seconds)
            continue

        patch_yolo_state(
            {
                "camera_ok": True,
                "yolo_status": "RUNNING",
                "error_message": "",
            }
        )

        try:
            while not dashboard_server.stop_event.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    set_yolo_error(
                        "CAMERA_READ_ERROR",
                        "Failed to read a frame from camera: {}".format(args.yolo_camera),
                        {"runtime_ready": True, "model_loaded": True},
                    )
                    break

                try:
                    results = model.predict(source=frame, **predict_kwargs)
                except Exception as exc:
                    patch_yolo_state(
                        {
                            "yolo_status": "INFERENCE_ERROR",
                            "camera_ok": True,
                            "error_message": "YOLO inference failed: {}".format(exc),
                        }
                    )
                    time.sleep(0.05)
                    continue

                if not results:
                    detection_patch = reset_detection_fields()
                else:
                    detection_patch = extract_detection_state(results[0], model)
                detection_patch["last_update_ms"] = current_time_ms()

                detection_patch.update(
                    {
                        "runtime_ready": True,
                        "model_loaded": True,
                        "camera_ok": True,
                        "yolo_status": "RUNNING",
                        "error_message": "",
                    }
                )
                patch_yolo_state(detection_patch)
        finally:
            try:
                capture.release()
            except Exception:
                pass

        if not dashboard_server.stop_event.is_set():
            time.sleep(retry_seconds)

    patch_yolo_state({"yolo_status": "STOPPED", "camera_ok": False})


class YoloDashboardHandler(dashboard_server.DashboardHandler):
    def do_GET(self) -> None:
        parsed = urlparse(self.path)

        if parsed.path == "/api/yolo/state":
            self.send_json(get_yolo_state_snapshot(), HTTPStatus.OK)
            return

        super().do_GET()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Jetson Nano dashboard with background YOLO test loop")
    parser.add_argument("--serial", default="/dev/ttyUSB0", help="Arduino serial device path")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind host")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP port")
    parser.add_argument("--mock", action="store_true", help="Use mock data instead of serial input")
    parser.add_argument("--config", default=str(dashboard_server.UNLOCK_CONFIG_PATH), help="Unlock config JSON path")

    parser.add_argument("--disable-yolo", action="store_true", help="Disable the background YOLO worker")
    parser.add_argument("--yolo-camera", default="/dev/video0", help="YOLO camera input. Default keeps the CSI alias behavior from camera_detection_test.py")
    parser.add_argument("--yolo-model", default="yolo11n.pt", help="YOLO model path or name. Example: yolo11n.pt or best.engine")
    parser.add_argument("--yolo-conf", type=float, default=0.25, help="YOLO confidence threshold")
    parser.add_argument("--yolo-imgsz", type=int, default=640, help="YOLO inference image size")
    parser.add_argument("--yolo-width", type=int, default=1280, help="Requested camera width")
    parser.add_argument("--yolo-height", type=int, default=720, help="Requested camera height")
    parser.add_argument("--yolo-fps", type=int, default=30, help="Requested camera framerate")
    parser.add_argument("--yolo-flip-method", type=int, default=0, help="GStreamer flip method for the CSI camera")
    parser.add_argument("--yolo-device", default="", help="Optional YOLO device override, for example cuda:0 or cpu")
    parser.add_argument("--yolo-retry-ms", type=int, default=2000, help="Retry interval after camera/runtime failures")
    return parser.parse_args()


def initialize_dashboard_state(args: argparse.Namespace) -> None:
    dashboard_server.stop_event.clear()
    dashboard_server.clear_serial_command_queue()

    dashboard_server.unlock_config = dashboard_server.load_unlock_config(Path(args.config))
    dashboard_server.bridge_mode = "mock" if args.mock else "serial"

    initial_state = {
        "bridge_mode": dashboard_server.bridge_mode,
        "bridge_status": "STARTING",
        "robot_id": dashboard_server.unlock_config.get("robot_id", "amr01"),
        "unlock_mode": dashboard_server.unlock_config.get("unlock_mode", "fixed"),
        "unlock_status": "IDLE",
        "unlock_message": "",
        "door_auto_close_ms": int(
            dashboard_server.unlock_config.get("door_auto_close_ms", dashboard_server.MOCK_UNLOCK_OPEN_MS)
        ),
        **dashboard_server.build_control_patch(),
        **dashboard_server.build_arm_patch(),
    }
    if args.mock:
        initial_state.update(
            {
                "mode_protocol_version": dashboard_server.MODE_PROTOCOL_VERSION,
                "current_mode": dashboard_server.RUN_MODE_AUTO,
                "current_auto_stage": "IDLE",
                "saved_auto_stage": "",
                "resume_pending": False,
                "manual_session_active": False,
                "task_active": False,
                "fault_latched": False,
                "resume_note": "No saved task",
                "selected_task": "",
            }
        )

    dashboard_server.patch_state(initial_state)
    dashboard_server.patch_task_selection_runtime({})
    replace_yolo_state(build_initial_yolo_state(args))


def main() -> None:
    args = parse_args()

    if not args.mock and dashboard_server.serial is None:
        raise SystemExit("pyserial is required. Install with: pip3 install -r requirements.txt")

    initialize_dashboard_state(args)

    bridge_worker = threading.Thread(
        target=dashboard_server.mock_reader_loop if args.mock else dashboard_server.serial_reader_loop,
        args=() if args.mock else (args.serial, args.baud),
        daemon=True,
    )
    bridge_worker.start()

    yolo_worker = threading.Thread(target=yolo_worker_loop, args=(args,), daemon=True)
    yolo_worker.start()

    server = dashboard_server.ThreadedHTTPServer((args.host, args.http_port), YoloDashboardHandler)
    print("[http] home ready at http://{}:{}/".format(args.host, args.http_port))
    print("[http] dashboard ready at http://{}:{}/dashboard".format(args.host, args.http_port))
    print("[http] unlock page ready at http://{}:{}/unlock".format(args.host, args.http_port))
    print("[http] yolo state ready at http://{}:{}/api/yolo/state".format(args.host, args.http_port))

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        dashboard_server.stop_event.set()
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()

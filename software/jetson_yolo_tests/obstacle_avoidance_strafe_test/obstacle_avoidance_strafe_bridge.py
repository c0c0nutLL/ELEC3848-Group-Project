import argparse
import json
import sys
import threading
import time


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Jetson-side YOLO bridge for the fixed-strafe obstacle avoidance prototype."
    )
    parser.add_argument("--camera", default="/dev/video0", help="CSI camera selector. Default: /dev/video0")
    parser.add_argument("--model", default="yolo11n.pt", help="YOLO model path or model name.")
    parser.add_argument("--serial", default="/dev/ttyACM0", help="Arduino serial device path.")
    parser.add_argument("--baud", type=int, default=115200, help="Arduino serial baud rate.")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference image size.")
    parser.add_argument("--width", type=int, default=1280, help="Requested CSI capture width.")
    parser.add_argument("--height", type=int, default=720, help="Requested CSI capture height.")
    parser.add_argument("--fps", type=int, default=30, help="Requested CSI camera framerate.")
    parser.add_argument("--flip-method", type=int, default=0, help="GStreamer nvvidconv flip-method.")
    parser.add_argument("--conf", type=float, default=0.20, help="Base YOLO confidence threshold.")
    parser.add_argument("--person-conf", type=float, default=0.35, help="Person confidence threshold.")
    parser.add_argument("--object-conf", type=float, default=0.35, help="Generic object confidence threshold.")
    parser.add_argument("--person-min-area", type=float, default=0.015, help="Minimum person box area ratio.")
    parser.add_argument("--object-min-area", type=float, default=0.020, help="Minimum generic object box area ratio.")
    parser.add_argument("--vision-hold-ms", type=int, default=500, help="Hold the last non-clear result briefly.")
    parser.add_argument("--report-interval-ms", type=int, default=800, help="Console report interval.")
    parser.add_argument("--yolo-device", default="", help="Optional YOLO device override, for example cuda:0.")
    parser.add_argument("--show", action="store_true", help="Show annotated camera output. Default is off.")
    parser.add_argument("--auto-start", action="store_true", help="Send START once after serial is ready.")
    parser.add_argument("--no-serial", action="store_true", help="Run vision only and do not open serial.")
    return parser.parse_args()


def import_dependencies(no_serial):
    try:
        import cv2  # type: ignore
    except ImportError:
        print("Missing dependency: cv2")
        sys.exit(1)

    try:
        from ultralytics import YOLO  # type: ignore
    except ImportError:
        print("Missing dependency: ultralytics")
        print("Run this script inside the Jetson YOLO Docker runtime.")
        sys.exit(1)

    serial_mod = None
    if not no_serial:
        try:
            import serial  # type: ignore
        except ImportError:
            print("Missing dependency: pyserial")
            print("Install it in the runtime that executes this script, for example: pip3 install pyserial")
            sys.exit(1)
        serial_mod = serial

    return cv2, YOLO, serial_mod


def parse_sensor_id(camera_path):
    camera_value = str(camera_path).strip()
    if camera_value in ("", "/dev/video0", "0"):
        return 0
    if camera_value.startswith("/dev/video") and camera_value[len("/dev/video"):].isdigit():
        return int(camera_value[len("/dev/video"):])
    if camera_value.isdigit():
        return int(camera_value)
    raise ValueError(
        "Unsupported camera input '{}'. Use /dev/video0, /dev/video1, or a numeric CSI sensor id.".format(
            camera_path
        )
    )


def open_camera(cv2, camera_path, width, height, fps, flip_method):
    sensor_id = parse_sensor_id(camera_path)
    pipeline = gstreamer_pipeline(
        sensor_id=sensor_id,
        capture_width=width,
        capture_height=height,
        display_width=width,
        display_height=height,
        framerate=fps,
        flip_method=flip_method,
    )
    return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)


def current_time_ms():
    return int(time.time() * 1000)


def zone_name(center_x, frame_width):
    if center_x < frame_width * 0.35:
        return "LEFT"
    if center_x > frame_width * 0.65:
        return "RIGHT"
    return "CENTER"


def score_candidate(confidence, area_ratio, zone):
    center_bonus = 0.08 if zone == "CENTER" else 0.03
    return confidence + min(area_ratio, 0.35) + center_bonus


def normalize_label(names, cls_id):
    if isinstance(names, dict):
        return str(names.get(cls_id, str(cls_id)))
    if isinstance(names, (list, tuple)) and 0 <= cls_id < len(names):
        return str(names[cls_id])
    return str(cls_id)


def choose_observation(result, frame_width, frame_height, args):
    observation = {
        "type": "CLEAR",
        "zone": "NONE",
        "confidence": 0,
        "label": "",
        "person_count": 0,
        "object_count": 0,
        "bbox": None,
    }

    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return observation

    frame_area = float(max(1, frame_width * frame_height))
    best_person = None
    best_object = None

    for box in boxes:
        cls_id = int(float(box.cls[0]))
        confidence = float(box.conf[0])
        label = normalize_label(result.names, cls_id)
        x1, y1, x2, y2 = [float(value) for value in box.xyxy[0].tolist()]
        width = max(0.0, x2 - x1)
        height = max(0.0, y2 - y1)
        area_ratio = (width * height) / frame_area
        zone = zone_name((x1 + x2) * 0.5, frame_width)
        candidate = {
            "label": label,
            "confidence": confidence,
            "confidence_pct": int(round(confidence * 100.0)),
            "zone": zone,
            "bbox": (int(x1), int(y1), int(x2), int(y2)),
            "score": score_candidate(confidence, area_ratio, zone),
        }

        if label == "person":
            observation["person_count"] += 1
            if confidence >= args.person_conf and area_ratio >= args.person_min_area:
                if best_person is None or candidate["score"] > best_person["score"]:
                    best_person = candidate
        else:
            observation["object_count"] += 1
            if confidence >= args.object_conf and area_ratio >= args.object_min_area:
                if best_object is None or candidate["score"] > best_object["score"]:
                    best_object = candidate

    if best_person is not None:
        observation.update(
            {
                "type": "PERSON",
                "zone": best_person["zone"],
                "confidence": best_person["confidence_pct"],
                "label": best_person["label"],
                "bbox": best_person["bbox"],
            }
        )
    elif best_object is not None:
        observation.update(
            {
                "type": "OBJECT",
                "zone": best_object["zone"],
                "confidence": best_object["confidence_pct"],
                "label": best_object["label"],
                "bbox": best_object["bbox"],
            }
        )

    return observation


def format_observation_command(observation):
    return "OBS {} {} {}".format(
        observation.get("type", "CLEAR"),
        observation.get("zone", "NONE"),
        int(observation.get("confidence", 0)),
    )


def apply_hold(raw_observation, last_nonclear, hold_ms):
    now_ms = current_time_ms()
    if raw_observation["type"] != "CLEAR":
        kept = dict(raw_observation)
        kept["held"] = False
        kept["ts_ms"] = now_ms
        return kept

    if last_nonclear and now_ms - int(last_nonclear.get("ts_ms", 0)) <= hold_ms:
        kept = dict(last_nonclear)
        kept["held"] = True
        return kept

    cleared = dict(raw_observation)
    cleared["held"] = False
    cleared["ts_ms"] = now_ms
    return cleared


def draw_overlay(cv2, frame, observation, fps_value):
    lines = [
        "FPS: {:.2f}".format(fps_value),
        "OBS: {} {} {}%".format(
            observation.get("type", "CLEAR"),
            observation.get("zone", "NONE"),
            int(observation.get("confidence", 0)),
        ),
        "HOLD: {}".format("YES" if observation.get("held") else "NO"),
    ]

    y = 30
    for line in lines:
        cv2.putText(
            frame,
            line,
            (15, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        y += 28


class SerialBridge(object):
    def __init__(self, serial_mod, port, baud):
        self._serial = serial_mod.Serial(port, baud, timeout=0.1)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop)
        self._thread.daemon = True
        self._last_summary = ""
        self._last_summary_ms = 0
        self._thread.start()

    def send(self, line):
        payload = "{}\n".format(line).encode("utf-8")
        self._serial.write(payload)
        self._serial.flush()

    def close(self):
        self._stop_event.set()
        try:
            self._thread.join(1.0)
        except RuntimeError:
            pass
        try:
            self._serial.close()
        except Exception:
            pass

    def _reader_loop(self):
        while not self._stop_event.is_set():
            try:
                raw_line = self._serial.readline()
            except Exception as exc:
                print("[serial] read error: {}".format(exc))
                return

            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("{"):
                self._handle_json_line(line)
            else:
                print("[arduino] {}".format(line))

    def _handle_json_line(self, line):
        try:
            packet = json.loads(line)
        except Exception:
            print("[arduino] {}".format(line))
            return

        if not isinstance(packet, dict):
            print("[arduino] {}".format(line))
            return

        summary = "state={} action={} front={} obs={} side={} fault={}".format(
            str(packet.get("state", "?")),
            str(packet.get("action", "?")),
            packet.get("front_cm"),
            str(packet.get("obstacle_type", "?")),
            str(packet.get("avoid_side", "NONE")),
            str(packet.get("fault", "")),
        )
        now_ms = current_time_ms()
        if summary != self._last_summary or now_ms - self._last_summary_ms >= 1000:
            print("[arduino] {}".format(summary))
            self._last_summary = summary
            self._last_summary_ms = now_ms


def main():
    args = parse_args()
    cv2, YOLO, serial_mod = import_dependencies(args.no_serial)

    try:
        capture = open_camera(cv2, args.camera, args.width, args.height, args.fps, args.flip_method)
    except ValueError as exc:
        print(str(exc))
        sys.exit(1)

    if not capture.isOpened():
        print("Failed to open CSI camera for input: {}".format(args.camera))
        sys.exit(1)

    serial_bridge = None
    if not args.no_serial:
        try:
            serial_bridge = SerialBridge(serial_mod, args.serial, args.baud)
        except Exception as exc:
            print("Failed to open serial {}: {}".format(args.serial, exc))
            sys.exit(1)

    if args.auto_start and serial_bridge is not None:
        serial_bridge.send("START")
        time.sleep(0.2)

    model = YOLO(args.model)
    predict_kwargs = {"conf": args.conf, "imgsz": args.imgsz, "verbose": False}
    if args.yolo_device:
        predict_kwargs["device"] = args.yolo_device

    if args.show:
        cv2.namedWindow("YOLO Strafe Obstacle Test", cv2.WINDOW_NORMAL)

    last_nonclear = None
    last_report_ms = 0
    last_command = ""
    previous_time = time.time()
    smoothed_fps = 0.0

    print("YOLO strafe obstacle bridge started.")
    print("Serial mode: {}".format("disabled" if args.no_serial else args.serial))
    print("Display mode: {}".format("ON" if args.show else "OFF"))

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                print("Failed to read a frame from camera: {}".format(args.camera))
                break

            results = model.predict(source=frame, **predict_kwargs)
            result = results[0]

            raw_observation = choose_observation(result, frame.shape[1], frame.shape[0], args)
            observation = apply_hold(raw_observation, last_nonclear, args.vision_hold_ms)
            if observation["type"] != "CLEAR":
                last_nonclear = dict(observation)

            command = format_observation_command(observation)
            if serial_bridge is not None:
                serial_bridge.send(command)

            now = time.time()
            delta = now - previous_time
            previous_time = now
            instant_fps = 0.0 if delta <= 0 else 1.0 / delta
            smoothed_fps = instant_fps if smoothed_fps <= 0.0 else smoothed_fps * 0.9 + instant_fps * 0.1

            now_ms = current_time_ms()
            if command != last_command or now_ms - last_report_ms >= args.report_interval_ms:
                print(
                    "[vision] fps={:.2f} cmd={} people={} objects={} label={} held={}".format(
                        smoothed_fps,
                        command,
                        observation.get("person_count", 0),
                        observation.get("object_count", 0),
                        observation.get("label", ""),
                        observation.get("held", False),
                    )
                )
                last_command = command
                last_report_ms = now_ms

            if args.show:
                annotated = result.plot()
                draw_overlay(cv2, annotated, observation, smoothed_fps)
                cv2.imshow("YOLO Strafe Obstacle Test", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if serial_bridge is not None:
            try:
                serial_bridge.send("STOP")
            except Exception:
                pass
            serial_bridge.close()
        capture.release()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

import argparse
import sys
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
        description="Run real-time YOLO detection on a camera stream and show FPS in the top-left corner."
    )
    parser.add_argument(
        "--camera",
        default="/dev/video0",
        help="Camera input selector. Default: /dev/video0, which maps to CSI sensor 0.",
    )
    parser.add_argument(
        "--model",
        default="yolo11n.pt",
        help="YOLO model path or name. Examples: yolo11n.pt, best.pt, best.engine",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="Confidence threshold.",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Inference image size.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="Requested camera width.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Requested camera height.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Requested CSI camera framerate.",
    )
    parser.add_argument(
        "--flip-method",
        type=int,
        default=0,
        help="GStreamer nvvidconv flip-method for the CSI camera.",
    )
    parser.add_argument(
        "--yolo-device",
        default="",
        help="Optional YOLO device override, for example cuda:0 or cpu.",
    )
    parser.add_argument(
        "--window-name",
        default="YOLO Camera Detection",
        help="OpenCV window title.",
    )
    return parser.parse_args()


def import_dependencies():
    try:
        import cv2  # type: ignore
    except ImportError:
        print("Missing dependency: cv2")
        print("Install OpenCV in the runtime that will execute this script.")
        sys.exit(1)

    try:
        from ultralytics import YOLO  # type: ignore
    except ImportError:
        print("Missing dependency: ultralytics")
        print("Use a compatible YOLO runtime or the Jetson Docker workflow for this script.")
        sys.exit(1)

    return cv2, YOLO


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


def draw_fps(cv2, frame, fps_value):
    label = "FPS: {:.2f}".format(fps_value)
    cv2.putText(
        frame,
        label,
        (15, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )


def main():
    args = parse_args()
    cv2, YOLO = import_dependencies()

    model = YOLO(args.model)
    try:
        capture = open_camera(
            cv2,
            args.camera,
            args.width,
            args.height,
            args.fps,
            args.flip_method,
        )
    except ValueError as exc:
        print(str(exc))
        sys.exit(1)

    if not capture.isOpened():
        print("Failed to open CSI camera for input: {}".format(args.camera))
        sys.exit(1)

    predict_kwargs = {
        "conf": args.conf,
        "imgsz": args.imgsz,
        "verbose": False,
    }
    if args.yolo_device:
        predict_kwargs["device"] = args.yolo_device

    cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)

    smoothed_fps = 0.0
    previous_time = time.time()

    print("Camera stream started. Press q or Esc to exit.")

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                print("Failed to read a frame from camera: {}".format(args.camera))
                break

            results = model.predict(source=frame, **predict_kwargs)
            annotated = results[0].plot()

            now = time.time()
            delta = now - previous_time
            previous_time = now

            instant_fps = 0.0
            if delta > 0:
                instant_fps = 1.0 / delta

            if smoothed_fps <= 0:
                smoothed_fps = instant_fps
            else:
                smoothed_fps = smoothed_fps * 0.9 + instant_fps * 0.1

            draw_fps(cv2, annotated, smoothed_fps)
            cv2.imshow(args.window_name, annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

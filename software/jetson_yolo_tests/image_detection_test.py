import argparse
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run YOLO on one local image, save the annotated result, and show it in a window."
    )
    parser.add_argument("--image", required=True, help="Path to the local input image.")
    parser.add_argument(
        "--model",
        default="yolo11n.pt",
        help="YOLO model path or name. Examples: yolo11n.pt, best.pt, best.engine",
    )
    parser.add_argument(
        "--output-dir",
        default="outputs",
        help="Directory for the saved annotated image.",
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
        "--yolo-device",
        default="",
        help="Optional YOLO device override, for example cuda:0 or cpu.",
    )
    parser.add_argument(
        "--window-name",
        default="YOLO Image Detection",
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


def build_output_path(image_path, output_dir):
    suffix = image_path.suffix if image_path.suffix else ".jpg"
    return output_dir / (image_path.stem + "_detected" + suffix)


def show_until_keypress(cv2, window_name, image):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, image)
    print("Press any key in the image window to close it.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    args = parse_args()
    cv2, YOLO = import_dependencies()

    image_path = Path(args.image).expanduser().resolve()
    if not image_path.exists():
        print("Input image not found: {}".format(image_path))
        sys.exit(1)

    image = cv2.imread(str(image_path))
    if image is None:
        print("Failed to read image: {}".format(image_path))
        sys.exit(1)

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = build_output_path(image_path, output_dir)

    model = YOLO(args.model)
    predict_kwargs = {
        "source": image,
        "conf": args.conf,
        "imgsz": args.imgsz,
        "verbose": False,
    }
    if args.yolo_device:
        predict_kwargs["device"] = args.yolo_device

    results = model.predict(**predict_kwargs)
    annotated = results[0].plot()

    if not cv2.imwrite(str(output_path), annotated):
        print("Failed to save annotated image: {}".format(output_path))
        sys.exit(1)

    print("Annotated image saved to: {}".format(output_path))
    show_until_keypress(cv2, args.window_name, annotated)


if __name__ == "__main__":
    main()

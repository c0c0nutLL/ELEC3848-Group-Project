# Jetson YOLO Test Scripts

These scripts are prepared for Jetson-side YOLO testing.

They are intentionally kept separate from:

- `software/jetson_dashboard`
- `software/jetson_dashboard/requirements.txt`

## Important Note

The current Jetson host environment is:

- Ubuntu 18.04
- Python 3.6.9

The latest `ultralytics` package does not run in that host Python directly.
These scripts are meant to be used in:

- a compatible YOLO runtime
- the future Jetson Docker workflow
- or any Python environment where `ultralytics` and `opencv-python` are available

## Files

- `image_detection_test.py`
  - Run detection on one local image
  - Save the annotated result image
  - Show the annotated result in a window

- `camera_detection_test.py`
  - Read live frames from `/dev/video0`
  - Show annotated detections in real time
  - Display FPS in the top-left corner

## Example Commands

Single image:

```bash
python3 image_detection_test.py \
  --model yolo11n.pt
```

If `--image` is omitted, the script uses `./people_test.jpg`.

USB camera:

```bash
python3 camera_detection_test.py \
  --camera /dev/video0 \
  --model yolo11n.pt
```

If `--camera` is omitted, the script uses `/dev/video0`.
In the current Jetson setup, that default is interpreted as CSI sensor `0` and opened through the GStreamer pipeline used in `software/camera_test.py`.

If you later export TensorRT, the same scripts can also use:

```bash
--model /path/to/best.engine
```

# Jetson YOLO Test Scripts

These scripts are prepared for Jetson-side YOLO testing.

They are intentionally kept separate from:

- `software/jetson_dashboard`
- `software/jetson_dashboard/requirements.txt`

## Current Status

The current Jetson host environment is:

- Ubuntu 18.04
- Python 3.6.9

- the Ultralytics YOLO runtime is already working through Docker
- the local host-side `venv` environment is still under construction and not ready yet
- current testing with `imshow` enabled in Docker gives about `8 FPS`
- `yolo26n.pt` and `yolo11n.pt` currently show similar FPS in this setup

The latest `ultralytics` package does not run in that host Python directly in a clean way.
For now, these scripts are primarily meant to be used in:

- the current Jetson Docker workflow
- or any Python environment where `ultralytics` and `opencv-python` are available

## Performance Note

Current observations:

- Docker + `imshow` is functional, but display-on FPS is still limited
- simply switching between `yolo26n.pt` and `yolo11n.pt` does not currently change FPS much
- future gains should be investigated from:
  - code-path optimization
  - display overhead reduction
  - model export/runtime optimization such as TensorRT
  - camera and preprocessing pipeline changes

## Files

- `image_detection_test.py`
  - Run detection on one local image
  - Save the annotated result image
  - Show the annotated result in a window

- `camera_detection_test.py`
  - Read live frames from the Jetson CSI camera
  - Show annotated detections in real time
  - Display FPS in the top-left corner

## Example Commands

Single image:

```bash
python3 image_detection_test.py \
  --model yolo11n.pt
```

If `--image` is omitted, the script uses `./people_test.jpg`.

CSI camera:

```bash
python3 camera_detection_test.py \
  --camera 0 \
  --model yolo11n.pt
```

If `--camera` is omitted, the script uses `/dev/video0`.
In the current Jetson setup, `/dev/video0` is treated as a CSI alias for sensor `0` and opened through the GStreamer pipeline used in `software/camera_test.py`.
This directory does not currently provide a separate USB camera code path.

If you later export TensorRT, the same scripts can also use:

```bash
--model /path/to/best.engine
```

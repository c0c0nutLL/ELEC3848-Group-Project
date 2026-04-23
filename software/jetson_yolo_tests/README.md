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

- `../jetson_dashboard/server_yolo_test.py`
  - integration test server that keeps the dashboard / HTTP / serial service running
  - reuses the same CSI camera open path and YOLO predict flow as `camera_detection_test.py`
  - does not call `imshow`
  - does not draw or save bounding-box images
  - exposes detection state through HTTP instead of a local display window

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

## Integrated UI + YOLO Test

For the current dashboard integration test, use:

```bash
python3 software/jetson_dashboard/server_yolo_test.py \
  --serial /dev/ttyUSB0 \
  --http-port 8080 \
  --yolo-camera /dev/video0 \
  --yolo-model yolo11n.pt
```

What it does:

- keeps the existing dashboard / UI and HTTP service available
- keeps the serial bridge structure from `software/jetson_dashboard/server.py`
- runs background camera YOLO detection in parallel
- exposes YOLO state from `GET /api/yolo/state`

The integrated test currently reports status such as:

- whether YOLO is enabled
- whether the runtime and model loaded successfully
- whether the camera is open
- whether anything was detected
- whether `person` was detected
- latest detected class list
- detection count
- top-confidence class and score
- latest update timestamp
- current error message, if any

Important constraints for this integrated path:

- no display is required
- no `imshow`
- no bounding-box rendering
- it is still expected to run inside the working Jetson Docker YOLO environment, or another equivalent runtime with OpenCV + Ultralytics available
- if the runtime is not available on the host Python environment, the HTTP server should still stay up and report the failure through the YOLO state API

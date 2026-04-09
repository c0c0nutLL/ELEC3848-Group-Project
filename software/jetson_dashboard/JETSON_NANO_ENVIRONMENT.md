# Jetson Nano Environment

Last updated: 2026-04-09

## Current Known Environment

- Target device: NVIDIA Jetson Nano
- OS: Ubuntu 18.04
- Python: 3.6.9
- PyTorch: installed and usable with CUDA
- `ultralytics`: not installed
- Existing Jetson-side project code:
  - `software/jetson_dashboard/server.py`
  - `software/jetson_dashboard/dashboard.html`

## Environment Notes For This Project

- The current Jetson-side dashboard code is lightweight and compatible with a simple Python environment.
- The current Jetson Python version is old for the latest Ultralytics package.
- We should treat "dashboard/serial bridge" and "YOLO inference runtime" as two separate concerns.

## YOLO Compatibility Notes

- Jetson Nano is a JetPack 4 device.
- The current `ultralytics` package requires Python 3.8 or newer.
- Therefore, the current host environment on this Nano cannot directly use the latest `pip install ultralytics` workflow.
- Ultralytics provides a JetPack 4 Docker image for Jetson, which is the official path for newer Ultralytics on Nano-class JetPack 4 systems.
- For deployment performance on Jetson, TensorRT should be preferred over plain PyTorch inference whenever possible.

## Recommended Direction For This Repo

### Keep Current Host Python For Existing Services

- Keep the existing host Python environment for:
  - serial bridge
  - dashboard server
  - task logging

### Run YOLO In A Separate Runtime

Use one of these approaches:

1. Preferred short-term option:
   Run YOLO in Docker on Jetson using the JetPack 4 Ultralytics image.
2. Preferred deployment option:
   Export a model for TensorRT and run inference on Jetson with TensorRT.
3. Avoid for now:
   Forcing the latest Ultralytics package into the current Python 3.6.9 host environment.

## Practical Next Steps

1. Verify the exact JetPack version on the Nano.
2. Verify the exact installed `torch` version.
3. Decide whether YOLO will be:
   - a separate Docker service, or
   - a TensorRT inference service integrated with the dashboard stack later.
4. When ready, pick a camera input:
   - USB camera, or
   - CSI camera.

## Useful Verification Commands On Jetson

```bash
python3 --version
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
pip3 show torch
dpkg-query --show nvidia-jetpack
cat /etc/nv_tegra_release
```

## Decision Summary

For this project, the safest path is:

- keep the existing Jetson host Python environment unchanged
- do not directly install the latest `ultralytics` into Python 3.6.9
- prefer Docker or TensorRT for YOLO on Jetson Nano

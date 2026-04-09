# YOLO Deployment Steps For Jetson Nano

Last updated: 2026-04-09

## Scope

This guide is written for the current project environment:

- Device: Jetson Nano
- OS: Ubuntu 18.04
- Python: 3.6.9 on host
- Host PyTorch: already installed and CUDA-enabled
- Host `ultralytics`: not installed

This guide assumes we will **not** install the latest `ultralytics` package into the host Python 3.6.9 environment.
Instead, YOLO runs in a separate Jetson Docker runtime or as a TensorRT engine generated from that runtime.

## Recommended Architecture

Keep the Jetson host environment for:

- dashboard server
- serial bridge to Arduino
- task logging

Run YOLO separately for:

- model testing
- model export
- camera inference

This separation avoids breaking the current `software/jetson_dashboard/server.py` stack.

## Why This Path

- Jetson Nano only supports JetPack 4.
- The current host Python is 3.6.9.
- The current `ultralytics` package requires Python 3.8 or newer.
- Ultralytics provides a JetPack 4 Docker image for Jetson.
- Ultralytics recommends TensorRT for best Jetson inference performance.

## Deployment Plan

There are two phases:

1. Phase A: bring up YOLO quickly in Docker.
2. Phase B: export to TensorRT and use the `.engine` model for deployment.

Do not skip Phase A. It is the fastest way to verify that the camera, GPU path, and model all work.

## Phase A: Quick Bring-Up In Docker

### Step 1: Verify Jetson base environment

Run these on the Jetson host:

```bash
python3 --version
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
dpkg-query --show nvidia-jetpack
cat /etc/nv_tegra_release
docker --version
sudo docker info
```

Expected result:

- Python stays at `3.6.9`
- `torch.cuda.is_available()` returns `True`
- JetPack is in the `4.x` family
- Docker is available

If Docker or NVIDIA runtime is missing, fix the JetPack / container runtime setup first before continuing.

### Step 2: Create a YOLO workspace on Jetson

Example:

```bash
mkdir -p ~/amr_yolo/models
mkdir -p ~/amr_yolo/data
mkdir -p ~/amr_yolo/runs
mkdir -p ~/amr_yolo/scripts
```

Use this directory for downloaded models, test images, exported engines, and custom scripts.

### Step 3: Pull the JetPack 4 Ultralytics container

Ultralytics documents this image for JetPack 4:

```bash
t=ultralytics/ultralytics:latest-jetson-jetpack4
sudo docker pull $t
```

### Step 4: Start the container

Start with a simple GPU-enabled container:

```bash
t=ultralytics/ultralytics:latest-jetson-jetpack4
sudo docker run -it --rm \
  --ipc=host \
  --runtime=nvidia \
  -v ~/amr_yolo:/workspace \
  --name amr-yolo \
  $t
```

If you want to use a USB camera later, add:

```bash
--device /dev/video0:/dev/video0
```

Start with a USB camera first. It is usually the lowest-friction path on Nano.

### Step 5: Verify Ultralytics inside the container

Inside the container:

```bash
python3 --version
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
yolo version
```

Expected result:

- Python inside the container is new enough for `ultralytics`
- CUDA is visible
- `yolo version` runs successfully

### Step 6: Run a first inference test

Inside the container:

```bash
cd /workspace
yolo predict model=yolo11n.pt source='https://ultralytics.com/images/bus.jpg'
```

This confirms:

- model download works
- Ultralytics runtime works
- GPU inference path works
- result output directory is created

If you want to follow the newest docs more closely, you can replace `yolo11n.pt` with `yolo26n.pt`.
For Jetson Nano, the smaller `n` model family is the correct place to start.

### Step 7: Test USB camera inference

Run the container again with the camera device mounted:

```bash
t=ultralytics/ultralytics:latest-jetson-jetpack4
sudo docker run -it --rm \
  --ipc=host \
  --runtime=nvidia \
  --device /dev/video0:/dev/video0 \
  -v ~/amr_yolo:/workspace \
  --name amr-yolo \
  $t
```

Inside the container:

```bash
cd /workspace
yolo predict model=yolo11n.pt source=0 imgsz=640
```

If `640` is too slow on Nano, reduce to:

```bash
yolo predict model=yolo11n.pt source=0 imgsz=512
```

or

```bash
yolo predict model=yolo11n.pt source=0 imgsz=416
```

## Phase B: Export TensorRT For Deployment

Once PyTorch inference works, move to TensorRT.
This is the recommended deployment format on Jetson.

### Step 8: Export a TensorRT engine

Inside the same container:

```bash
cd /workspace
yolo export model=yolo11n.pt format=engine imgsz=640 half=True
```

This should create a TensorRT engine such as:

```bash
yolo11n.engine
```

For Nano, `FP16` is the practical first choice.
It usually gives a better speed/accuracy tradeoff than raw PyTorch on Jetson.

### Step 9: Run inference with the exported engine

Still inside the container:

```bash
yolo predict model=yolo11n.engine source='https://ultralytics.com/images/bus.jpg'
```

Then test the USB camera:

```bash
yolo predict model=yolo11n.engine source=0 imgsz=640
```

At this point, deployment is real:

- the model is running on Jetson
- inference no longer depends on plain PyTorch weights
- the runtime path is closer to what you would ship in the demo

## Phase C: Custom Model For This Project

For this project, custom training should usually happen on a stronger machine, not on the Nano.

Recommended flow:

1. Collect and label images on another machine.
2. Train the model there.
3. Export or copy the best `.pt` model to Jetson.
4. On Jetson, export that model to TensorRT:

```bash
yolo export model=best.pt format=engine imgsz=640 half=True
```

5. Deploy `best.engine` on the Nano.

This avoids turning the Jetson Nano into a training machine.

## Suggested Runtime Layout For This Repo

Keep the current dashboard code separate from YOLO runtime:

- host side:
  - `software/jetson_dashboard/server.py`
  - serial bridge
  - dashboard
- YOLO side:
  - Docker container
  - camera inference
  - TensorRT engine generation

Later, connect them using a simple boundary:

- option A: YOLO writes JSON lines to a local file
- option B: YOLO sends HTTP POST to the dashboard server
- option C: YOLO publishes a local message queue event

For this project, option A or B is enough.

## Practical Model Selection

Start with:

- `yolo11n.pt`

Only move upward if accuracy is clearly insufficient:

- `yolo11s.pt`

Avoid larger models on Nano unless a real test shows you still have enough FPS.

## Performance Tuning Checklist

Before benchmarking on Jetson host, run:

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

Optional monitoring:

```bash
sudo pip install jetson-stats
sudo reboot
jtop
```

## What Not To Do

- Do not directly `pip install ultralytics` into the host Python 3.6.9 environment.
- Do not merge YOLO dependencies into `software/jetson_dashboard/requirements.txt`.
- Do not start with a large model on Nano.
- Do not optimize for CSI camera first. Bring up a USB camera first unless CSI is mandatory.

## Minimum Success Criteria

This YOLO bring-up is considered complete when all of the following work:

1. Docker container starts with NVIDIA runtime.
2. `yolo predict model=yolo11n.pt source=...` runs successfully.
3. `yolo export model=yolo11n.pt format=engine half=True` completes successfully.
4. `yolo predict model=yolo11n.engine source=0` works with a USB camera.

After that, integration with the project dashboard can begin.

## Sources

- Ultralytics Jetson guide:
  - https://docs.ultralytics.com/guides/nvidia-jetson/
- Ultralytics PyPI package metadata:
  - https://pypi.org/project/ultralytics/
- NVIDIA PyTorch for Jetson release notes:
  - https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html
- NVIDIA Installing PyTorch for Jetson:
  - https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html

# Isaac ROS Perceptor for RealSense D456

A ROS 2 Humble perception stack combining **cuVSLAM + NvBlox + RealSense D456** on **NVIDIA Jetson AGX Orin 64GB**.

NVIDIA's official [Isaac Perceptor](https://github.com/NVIDIA-ISAAC-ROS/isaac_perceptor) targets the Nova sensor suite (Hawk/Owl cameras), which does not support Intel RealSense. This package fills that gap by providing a D456-optimized perception pipeline with the same architectural patterns (YAML-driven configuration via `rgbd_perceptor`).

## Features

- **4 mapping modes**: static, dynamic, people\_detection, people\_segmentation
- **Splitter toggle** (`use_splitter`): 15fps clean IR (low-speed) or 30fps with denoising (high-speed up to 80km/h)
- **D456-specific optimizations**: 848x480x30 depth, QoS fix for NITROS zero-copy, cuVSLAM jitter threshold tuning, IMU disabled (FW limitation)
- **DNN integration**: PeopleNet detection + PeopleSemSegNet ShuffleSeg segmentation with auto-resize (848x480 to 640x480)
- **YAML-driven config**: All topic remappings and parameters in YAML files, no hardcoded Python remappings
- **Deploy container**: Single `run_deploy.sh` command with Makefile shortcuts

## Hardware Requirements

| Component | Spec |
|-----------|------|
| Platform | NVIDIA Jetson AGX Orin 64GB |
| Camera | Intel RealSense D456 (FW 5.15.0.2+) |
| Isaac ROS | Release 3.2 (Humble) |
| JetPack | 6.x (L4T r36.4) |

## Quick Start

```bash
cd /mnt/nova_ssd/workspaces/isaac_ros-dev

# Using Makefile (recommended)
make              # Static 3D reconstruction (default)
make dynamic      # Dynamic scene reconstruction
make fast         # High-speed driving (30fps, ground constraint)
make detect       # People detection (PeopleNet)
make segment      # People segmentation (ShuffleSeg)
make shell        # Interactive debug shell

# Or using run_deploy.sh directly
src/isaac_ros_common/scripts/run_deploy.sh
src/isaac_ros_common/scripts/run_deploy.sh mode:=dynamic
src/isaac_ros_common/scripts/run_deploy.sh mode:=dynamic use_splitter:=False enable_ground_constraint:=True
src/isaac_ros_common/scripts/run_deploy.sh mode:=people_detection
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `mode` | `static` | `static`, `dynamic`, `people_segmentation`, `people_detection` |
| `use_splitter` | `True` | `True`: 15fps clean IR (low-speed). `False`: 30fps with dot pattern (high-speed) |
| `enable_ground_constraint` | `False` | Constrain cuVSLAM to 2D plane (vehicles) |
| `run_rviz` | `True` | Launch RViz2 visualization |
| `people_segmentation` | `shuffleseg` | `shuffleseg` (Jetson recommended) or `vanilla` |
| `camera_serial_numbers` | (auto) | Specific D456 serial number |
| `rosbag` | `None` | Path to rosbag for replay |
| `enable_imu_fusion` | `False` | Experimental: D456 IMU has no factory calibration |

## Architecture

```
perceptor_realsense_d456.launch.py    (thin orchestrator)
  |
  +-- sensors/realsense_d456.launch.py  (D456 driver + splitter)
  |
  +-- rgbd_perceptor.launch.py          (YAML-driven nvblox + cuVSLAM)
  |     \-- params/d456_perceptor_*.yaml  (mode/splitter config selection)
  |
  +-- detection/segmentation pipeline   (DNN, delayed 10s for camera init)
  |
  +-- rviz2                             (visualization)
```

### YAML Configuration Files

| File | Use Case |
|------|----------|
| `params/d456_perceptor_splitter_on.yaml` | Static/dynamic, 15fps clean IR |
| `params/d456_perceptor_splitter_off.yaml` | High-speed 30fps, denoising enabled |
| `params/d456_perceptor_people_detection.yaml` | PeopleNet with 640x480 resize |
| `params/d456_perceptor_people_segmentation.yaml` | ShuffleSeg segmentation |

## D456-Specific Adaptations

### vs D435i (default in nvblox\_examples)

| | D435i | D456 (this package) |
|---|---|---|
| Depth resolution | 640x480x60 | **848x480x30** |
| Splitter IR FPS | 30 | **15** |
| cuVSLAM jitter threshold | 34ms | **100ms** (splitter on) / 34ms (splitter off) |
| PeopleNet input | Native 640x480 | **ResizeNode 848x480 to 640x480** |
| IMU | Working | **Disabled** (FW 5.15.0.2 limitation) |
| QoS (splitter output) | SENSOR\_DATA | **SYSTEM\_DEFAULT** (NITROS compatible) |

### Splitter ON vs OFF

| | Splitter ON (default) | Splitter OFF |
|---|---|---|
| IR FPS for cuVSLAM | 15 fps (clean, no dots) | 30 fps (dot pattern + denoising) |
| Suitable speed | Low-speed (parking, campus) | **High-speed (up to 80km/h)** |
| Depth source | Splitter output | Raw camera topic |
| `enable_image_denoising` | false | true |

## DNN Model Setup (one-time)

People detection/segmentation modes require TensorRT engine files. Install once:

```bash
make shell
# Inside container:
ros2 run isaac_ros_peoplesemseg_models_install install_peoplesemsegnet_shuffleseg.sh --eula
ros2 run isaac_ros_peoplenet_models_install install_peoplenet_amr_rs.sh --eula
```

Models are cached on the host at `/mnt/nova_ssd/isaac_ros_assets/` and persist across container restarts.

## Commit History

| Commit | Description |
|--------|-------------|
| `84f7b46` | Initial commit: package structure, D456 camera configs, cuVSLAM + NvBlox + RViz |
| `3b2e9a2` | Add multi-mode support (static, dynamic, people\_segmentation, people\_detection) |
| `deba21b` | Add `use_splitter` option for high-speed 30fps IR mode |
| `4c76731` | Fix people\_detection: ResizeNode for D456 848x480 to PeopleNet 640x480 |
| `eb4e546` | RViz config: add detection/segmentation/dynamic image display panels |
| `e682fc2` | Refactor to YAML-driven config using `rgbd_perceptor` pattern from isaac\_perceptor |

## Related Repositories

- [Hyeonvidia/isaac\_ros\_common](https://github.com/Hyeonvidia/isaac_ros_common/tree/feature/perceptor-realsense-d456) — `run_deploy.sh`, deploy scripts
- [NVIDIA-ISAAC-ROS/isaac\_ros\_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common) — Upstream (release-3.2)
- [NVIDIA-ISAAC-ROS/isaac\_perceptor](https://github.com/NVIDIA-ISAAC-ROS/isaac_perceptor) — Official Nova perceptor (rgbd\_perceptor pattern reference)

## License

Apache-2.0

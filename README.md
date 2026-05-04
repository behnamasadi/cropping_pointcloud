# Cropping Pointcloud

[![License: BSD-3-Clause](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)
[![ROS 2: Humble | Jazzy](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blueviolet)](#tested-platforms)
[![CI](https://github.com/behnamasadi/cropping_pointcloud/actions/workflows/ci.yml/badge.svg)](https://github.com/behnamasadi/cropping_pointcloud/actions/workflows/ci.yml)

A ROS 2 node that crops an incoming `sensor_msgs/PointCloud2` against an axis-aligned bounding box (six tunable bounds) and optionally extracts the dominant non-planar object via RANSAC plane removal. The cropping volume is **live-tunable** at runtime — drag the rqt sliders, see the cropped cloud update in RViz immediately.

| Source scene (Gazebo) | Live-tunable bounds (rqt) |
|---|---|
| ![Gazebo: gpu_lidar_sensor.sdf with Playground + obstacle box + model_with_lidar](images/gazebo_scene.png) | ![rqt Dynamic Reconfigure with sliders for x/y/z bounds and RANSAC params](images/rqt_gui.png) |

| `/camera/rgb/points` (raw) | `/cropped_cloud` (AABB-clipped) | `/object_cloud` (after RANSAC plane removal) |
|---|---|---|
| ![raw lidar in RViz](images/raw.png) | ![cropped lidar in RViz](images/cropped.png) | ![object cloud in RViz](images/object.png) |

> The original ROS 1 (Kinetic/Noetic) implementation is preserved as the git tag **`ros1-legacy`**.

---

## Architecture

**Two units.** The data source publishes `sensor_msgs/PointCloud2`; the cropper unit subscribes, filters, and shows the result. They communicate over DDS exactly the way a real distributed ROS 2 system would — same `network_mode: host`, same `ipc: host`, same `ROS_DOMAIN_ID`. To split across actual machines later, run the cropper on one box and the data-source on another, keeping `ROS_DOMAIN_ID` identical on both.

```
┌──────────────────────────────────┐   /camera/rgb/points    ┌──────────────────────────────────────┐
│ gazebo (data source)             │ ───────────────────────►│ cropper (perception + GUI)           │
│   ─ Gazebo (Fortress / Harmonic) │   PointCloud2  (DDS)    │   ─ cropping_pointcloud node         │
│     headless sim w/ GPU LiDAR    │                         │     (publishes /cropped_cloud,       │
│   ─ ros_gz_bridge                │                         │      /object_cloud)                  │
│                                  │                         │   ─ RViz2 (preset layout)            │
│ swap with: real sensor driver,   │                         │   ─ rqt Dynamic Reconfigure sliders  │
│ rosbag2 replay, etc.             │                         │                                      │
└──────────────────────────────────┘                         └──────────────────────────────────────┘
        docker/gazebo.Dockerfile                                     docker/cropper.Dockerfile
```

| Service | Image | Purpose |
|---|---|---|
| `gazebo`  | `cropping_pointcloud/gazebo`  | Headless Gazebo (`gpu_lidar_sensor.sdf`) + `parameter_bridge`. Emits `sensor_msgs/PointCloud2` on `/camera/rgb/points`. Replace with any publisher of the same topic. |
| `cropper` | `cropping_pointcloud/cropper` | One launch file (`cropping_demo.launch.py`) starts: the cropping node (publishes `/cropped_cloud` + `/object_cloud`), RViz2 with the bundled layout, rqt Dynamic Reconfigure pinned to the node. Built on `osrf/ros:*-desktop-full` so PCL + RViz are already present — only `rqt_reconfigure` is added. |

## Tested platforms

| `ROS_DISTRO` | Ubuntu | PCL | Gazebo | apt package | CLI |
|---|---|---|---|---|---|
| **`humble`** (LTS, 2022 → 2027) | 22.04 (Jammy) | 1.12 | **Fortress** | `ignition-fortress` | `ign gazebo` |
| **`jazzy`** (LTS, 2024 → 2029) | 24.04 (Noble) | 1.14 | **Harmonic** | `gz-harmonic` | `gz sim` |

> Iron and Garden are skipped — Iron is EOL since Nov 2024; Garden was a transitional Gazebo release. Humble and Jazzy cover both active LTS users.

The `.env` file at the repo root selects the pairing (default: Humble + Fortress). Switch by uncommenting the Jazzy block:

```ini
# .env
ROS_DISTRO=humble
GZ_PKG=ignition-fortress
GZ_CMD=ign gazebo

# ROS_DISTRO=jazzy
# GZ_PKG=gz-harmonic
# GZ_CMD=gz sim

ROS_DOMAIN_ID=42
```

---

## Quick start

```bash
# 1. clone
git clone https://github.com/behnamasadi/cropping_pointcloud.git
cd cropping_pointcloud

# 2. allow X11 from containers (once per host login session)
xhost +local:docker

# 3. build the two images
#    - cropper: ~30 s (FROM osrf/ros:*-desktop-full, only adds rqt_reconfigure + colcon build)
#    - gazebo:  ~5 min first time (Ignition Fortress is the slow apt install)
docker compose build

# 4. bring up the stack
docker compose up -d        # detached, so the same shell is free to inspect topics
docker compose logs -f      # tail logs from both services (Ctrl+C to stop tailing)
```

**Three windows pop up on your screen:**

1. **Gazebo (the data source)** — 3D view of `gpu_lidar_sensor.sdf`: a `model_with_lidar` chassis at world (4, 0, 0.5) facing −X, a 1 m³ obstacle box at (0, −1, 0.5), a small box at (0.05, 0.05, 0.05), and the OpenRobotics Playground model at the origin. The simulation is already unpaused.
2. **RViz2 (the consumer)** — preset layout from `rviz/cropping_pointcloud.rviz`. Fixed Frame is `model_with_lidar/link/gpu_lidar`. Three PointCloud2 displays already added: white = `/camera/rgb/points` (raw lidar), red = `/cropped_cloud`, green = `/object_cloud` after RANSAC plane removal.
3. **rqt Dynamic Reconfigure (the operator console)** — pinned to the `/cropping_pointcloud` node. Sliders for `x_min`/`x_max`/`y_*`/`z_*`/`plane_distance_threshold`/`plane_max_iterations`. Drag a slider; the red cloud in RViz reshapes immediately.

### Verify the data flow

```bash
# topics that exist (run from a host shell while the stack is up)
docker compose exec cropper bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic list'

# rate (should be ~10 Hz)
docker compose exec cropper bash -lc \
  'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic hz /cropped_cloud'

# point counts at default bounds (x: [-2, 8], y: [-5, 5], z: [-1, 2])
docker compose exec cropper bash -lc \
  'source /opt/ros/humble/setup.bash &&
   for t in /camera/rgb/points /cropped_cloud /object_cloud; do
     echo -n "$t  width="
     timeout 3 ros2 topic echo $t --once --field width 2>/dev/null | head -1
   done'
```

Expected:
```
/camera/rgb/points  width=640        (× height=16 = 10 240 points, organized lidar)
/cropped_cloud      width=6 800-ish  (after AABB crop in lidar frame)
/object_cloud       width=3 500-ish  (after RANSAC removes the ground plane)
```

Tighten a bound to confirm the live tuning loop:

```bash
docker compose exec cropper bash -lc \
  'source /opt/ros/humble/setup.bash &&
   ros2 param set /cropping_pointcloud x_max 2.0 &&
   ros2 param set /cropping_pointcloud x_min 0.0 &&
   sleep 1 &&
   timeout 3 ros2 topic echo /cropped_cloud --once --field width | head -1'
# → ~2 200, the red cloud in RViz visibly shrinks
```

Tear down:

```bash
docker compose down               # stop both containers
docker compose down --rmi local   # also delete the two locally-built images
```

### GPU acceleration

The compose file requests an NVIDIA GPU on both services (helps lidar sensor rendering and dense-cloud display in RViz). Requires the host to have:

1. The NVIDIA driver installed.
2. The [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/) installed and configured for Docker.

Without an NVIDIA GPU, comment out the `deploy:` blocks in `docker-compose.yml`. Sensor rendering falls back to Mesa / software; everything still runs.

---

## Configuration

### `.env`

| Variable | Default | What it does |
|---|---|---|
| `ROS_DISTRO` | `humble` | Picks the base images. `gazebo` uses `ros:${ROS_DISTRO}-ros-base`; `cropper` uses `osrf/ros:${ROS_DISTRO}-desktop-full` (which already ships PCL + RViz). |
| `GZ_PKG` | `ignition-fortress` | Gazebo apt package. |
| `GZ_CMD` | `ign gazebo` | CLI to launch Gazebo. Becomes `gz sim` on Jazzy. |
| `ROS_DOMAIN_ID` | `42` | DDS partition; both services use the same value. |

### Cropping bounds (the `cropper` unit)

Defaults are baked into `launch/cropping_demo.launch.py` and match the Gazebo `gpu_lidar_sensor.sdf` world geometry:

```python
parameters=[{
    "x_min": -2.0, "x_max": 8.0,
    "y_min": -5.0, "y_max": 5.0,
    "z_min": -1.0, "z_max": 2.0,
}]
```

Bounds are exposed with `FloatingPointRange` descriptors (range `[-10.0, 10.0] m`, step `0.01 m`), which is what makes rqt render sliders instead of text boxes.

### Headless mode (no display)

The launch file accepts `gui:=false` to skip RViz + rqt — useful when running the cropper on a headless robot:

```yaml
# in docker-compose.yml, override the cropper command:
command: ros2 launch cropping_pointcloud cropping_demo.launch.py gui:=false
```

### Live tuning

Two equivalent ways while the stack is up:

```bash
# rqt is already running — drag the slider for x_max, y_min, etc.

# Or from any shell on the host:
docker compose exec cropper bash -c \
  'ros2 param set /cropping_pointcloud x_max 3.0'

docker compose exec cropper bash -c \
  'ros2 param dump /cropping_pointcloud > /tmp/tuned.yaml'   # save current values
```

---

## Splitting across machines (real-world deployment)

The compose file is one rehearsal of a multi-host system. The two units can run on different physical machines:

1. **Data-source box** (a robot, an NVIDIA Jetson, the Gazebo machine, …) — runs the `gazebo` service (or your real sensor driver).
2. **Operator workstation** (your laptop, a desktop) — runs the `cropper` service (which includes RViz + rqt).

Put the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` on every machine. As long as DDS multicast traverses the network (or you configure unicast peer discovery), the cropper sees the data source — that's the whole point of ROS 2's transport layer. The cropping node subscribes to `/camera/rgb/points` regardless of which physical box publishes it.

> **Multicast caveat:** corporate switches and Wi-Fi often drop multicast. The standard fix is either Cyclone DDS with explicit `CYCLONEDDS_URI` peer config, or [`rmw_zenoh`](https://github.com/ros2/rmw_zenoh)'s discovery server.

---

## Topics

| Direction | Topic | Type | QoS |
|---|---|---|---|
| in  | `camera/rgb/points` (default; remappable via `input_topic` param) | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS` |
| out | `cropped_cloud` | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS` |
| out | `object_cloud` | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS` (only when `extract_object=true`) |

## Parameters

| Name | Type | Default | Range / step | Description |
|---|---|---|---|---|
| `input_topic` | string | `camera/rgb/points` | — | Input point cloud topic |
| `frame_id` | string | `""` | — | Override the output `frame_id` (empty = keep input frame) |
| `extract_object` | bool | `true` | — | Run RANSAC plane removal on the cropped cloud |
| `plane_distance_threshold` | double | `0.02` | `[0.001, 0.5]` step `0.001` | RANSAC inlier distance threshold (m) |
| `plane_max_iterations` | int | `100` | `[10, 1000]` step `1` | RANSAC iteration cap |
| `x_min`, `x_max` | double | `-0.44`, `0.30` | `[-10.0, 10.0]` step `0.01` | Crop bounds along X (m) |
| `y_min`, `y_max` | double | `-1.00`, `0.24` | `[-10.0, 10.0]` step `0.01` | Crop bounds along Y (m) |
| `z_min`, `z_max` | double | `-1.00`, `1.08` | `[-10.0, 10.0]` step `0.01` | Crop bounds along Z (m) |

All bounds are live-updatable and take effect on the next callback.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `cannot connect to display :0` | Re-run `xhost +local:docker` on the host (persists across container restarts but resets on host logout/reboot). |
| RViz black or crashes on GL init | NVIDIA Container Toolkit not configured. Verify `nvidia-smi` works inside `docker run --rm --gpus all ubuntu:22.04 nvidia-smi`. Last resort: `LIBGL_ALWAYS_SOFTWARE=1` in `rviz`'s environment. |
| `parameter_bridge` runs but `ros2 topic hz /camera/rgb/points` is silent | Gazebo started paused. The entrypoint passes `-r` so this shouldn't happen — check `docker compose logs gazebo` for crashes, particularly libEGL warnings that escalate to errors. |
| Cropped cloud is empty even with wide bounds | The lidar frame_id is `model_with_lidar/link/gpu_lidar`; bounds are interpreted in **that** frame, not world. Sensor is at world (4, 0, 0.5) facing −X, so the obstacle box at world (0, −1, 0.5) sits at lidar-local x≈4. The defaults already account for this. |
| rqt slider exists but moving it does nothing | The on-set callback only fires after the cropper finishes initializing. `docker compose logs cropper` should show `subscribed to 'camera/rgb/points'…` before tuning works. |
| Two containers on different hosts can't see each other | DDS multicast doesn't traverse most switches. Set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and a `CYCLONEDDS_URI` peer file, or use `rmw_zenoh`. |

---

## Building from source on the host (no Docker)

If you'd rather not use Docker:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/behnamasadi/cropping_pointcloud.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select cropping_pointcloud --symlink-install
source install/setup.bash
ros2 run cropping_pointcloud cropping_pointcloud
```

You'll need ROS 2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04) on the host.

---

## License

BSD-3-Clause. See [`LICENSE`](LICENSE).

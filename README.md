# megarover-v3-fastlio2

**Indoor localization & mapping with a Vstone MegaRover V3 using FAST-LIO² (Livox Mid-360) fused with the rover's wheel odometry.**

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)
![Lidar](https://img.shields.io/badge/LiDAR-Livox%20Mid--360-brightgreen)
![SLAM](https://img.shields.io/badge/SLAM-FAST--LIO2-blue)
![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)

[![MegaRover V3 + Livox Mid-360](https://github.com/alfinjunaedy/megarover-v3-fastlio2/raw/main/imgs/mobilerobot.jpg)](https://github.com/alfinjunaedy/megarover-v3-fastlio2/blob/main/imgs/mobilerobot.jpg)

A complete ROS 2 (Humble) stack that runs **FAST-LIO²** with a **Livox Mid-360**
(points + built-in IMU) mounted on a **Vstone MegaRover Ver.3.0**. The
LiDAR-inertial odometry (`/Odometry`) is re-published by a small bridge node as
`/fastlio2_pose` (a `geometry_msgs/PoseStamped`) so the rover controller can be
reused **unchanged** from the previous **ORB-SLAM3 (RGB-D)** version of this
project. One launch file brings up the whole system.

This is the LiDAR successor to [megarover-v3-orbslam3](https://github.com/alfinjunaedy/megarover-v3-orbslam3) —
same robot and same controller, but a solid-state LiDAR-inertial SLAM instead of a
camera. The only difference on the controller side is the pose topic:
`/orbslam_pose` → `/fastlio2_pose`.

| Real robot | Trajectory from experiment |
| --- | --- |
| [![robot](https://github.com/alfinjunaedy/megarover-v3-fastlio2/raw/main/imgs/mobilerobot.gif)](https://github.com/alfinjunaedy/megarover-v3-fastlio2/blob/main/imgs/mobilerobot.gif) | [![trajectory](https://github.com/alfinjunaedy/megarover-v3-fastlio2/raw/main/imgs/trajectory.jpg)](https://github.com/alfinjunaedy/megarover-v3-fastlio2/blob/main/imgs/trajectory.jpg) |

* * *

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware](#hardware)
- [System Architecture](#system-architecture)
- [Workspace Layout](#workspace-layout)
- [Repository Contents](#repository-contents)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [0\. ROS 2 Humble + dependencies](#0-ros-2-humble--dependencies)
  - [1\. Livox SDK2](#1-livox-sdk2)
  - [2\. Livox ROS 2 driver](#2-livox-ros-2-driver)
  - [3\. FAST-LIO2](#3-fast-lio2)
  - [4\. This repository → `~/fastlio_ws` + `~/ros2_ws`](#4-this-repository--fastlio_ws--ros2_ws)
  - [5\. Environment setup](#5-environment-setup)
- [Configuration — Network & IPs](#configuration--network--ips)
- [Usage](#usage)
- [Topics](#topics)
- [Rover Motion Programming](#rover-motion-programming)
- [Troubleshooting](#troubleshooting)
- [License](#license)
- [Acknowledgments](#acknowledgments)

* * *

## Overview

The MegaRover V3 base runs Vstone's **micro-ROS firmware**, exposing the rover's
sensors and motor interface to the ROS 2 network through a serial micro-ROS Agent
(115200 baud). On top of that, this project adds LiDAR-based localization and
mapping:

1. **Livox Mid-360** streams a 3D point cloud and a built-in 9-axis IMU over UDP
   (`livox_ros_driver2`).
2. **FAST-LIO²** fuses the LiDAR points with the IMU in a tightly-coupled
   iterated Kalman filter and publishes `/Odometry`, `/path`, `/cloud_registered`
   and `/Laser_map` (HKU-MARS, ROS 2 port by Ericsii).
3. **`fastlio2_pose`** re-publishes FAST-LIO²'s `/Odometry` as a
   `geometry_msgs/PoseStamped` on `/fastlio2_pose` (so the controller is identical
   to the ORB-SLAM3 version).
4. **`rover_controller`** computes the rover odometry (`/rover_odo`, `/tf`) and
   drives the base (`/rover_twist`) toward a scripted route, using the
   `/fastlio2_pose` estimate as feedback.

## Features

- 🎯 **LiDAR-inertial SLAM** — FAST-LIO² on a Livox Mid-360, no GPS, works indoors
- 🗺️ **Online global mapping** — incremental ikd-Tree map (`/Laser_map`) in RViz
- 🤖 **Drop-in controller swap** — `fastlio2_pose` keeps the PoseStamped interface
  of the previous ORB-SLAM3 version, so `rover_controller` is reused unchanged
- 🧭 **360° solid-state LiDAR** — non-repeating Mid-360 pattern + built-in IMU
- 🚀 **Single launch file** — staggered bring-up of agent → driver → SLAM → pose
  node → RViz → controller
- 🔨 **Powerful re-config** — the whole pipeline is tuned from one `mid360.yaml`

## Hardware

| Component | Notes |
| --- | --- |
| **Vstone MegaRover Ver.3.0** | Differential-drive base, micro-ROS firmware (ESP32), serial @ 115200 |
| **Livox Mid-360** | Solid-state LiDAR + built-in 9-axis IMU, UDP / PTP time sync, 192.168.1.114 |
| **Onboard Mini PC** | Ubuntu 22.04 x86\_64 — FAST-LIO2 needs ≥ 8 GB RAM |
| Network | Ethernet between the Mid-360 (`,114`) and the PC (`.50`) on the `192.168.1` subnet |

## Workspace Layout

The project is spread across three workspaces (each has its own build/lifecycle):

| Path | Purpose | Built with |
| --- | --- | --- |
| `~/fastlio_ws` | Livox driver + FAST-LIO2 + `fastlio2_pose` bridge | `colcon` |
| `~/ros2_ws` | Rover workspace: `rover_control` (odometry + bring-up launch) | `colcon` |
| `~/uros_ws` | micro-ROS Agent workspace for the MegaRover V3 base (Vstone original setup) | `colcon` + `micro_ros_setup` |

```
~/
├── Livox-SDK2/            # clone from Livox-SDK (step 1)
├── fastlio_ws/            # ROS 2 — LiDAR + SLAM
│   └── src/
│       ├── livox_ros_driver2/     # Livox Mid-360 driver (step 2)
│       ├── FAST_LIO_ROS2/         # FAST-LIO2 algorithm (step 3)
│       │   └── config/mid360.yaml <- from this repo (step 4)
│       └── fastlio2_pose/         # <- from this repo (step 4)
├── ros2_ws/               # ROS 2 — rover
│   └── src/
│       └── rover_control/         # <- from this repo (step 4; launch + rviz)
└── uros_ws/               # micro-ROS agent (step 0)
    └── src/micro_ros_setup/
```

## Repository Contents

```
megarover-v3-fastlio2/
├── fastlio_ws/
│   └── src/
│       ├── fastlio2_pose/
│       │   ├── src/fastlio2_pose_node.cpp   # /Odometry -> /fastlio2_pose
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       └── FAST_LIO_ROS2/
│           └── config/mid360.yaml           # tuned for the MegaRover V3
├── ros2_ws/
│   └── src/
│       └── rover_control/
│           ├── launch/rover_bringup_launch.py  # full-system bring-up
│           ├── rviz/rviz_config.rviz           # RViz config loaded by the launch
│           ├── src/rover_controller.cpp        # odometry + planner (uses /fastlio2_pose)
│           ├── CMakeLists.txt
│           └── package.xml
├── imgs/
│   ├── mobilerobot.gif    # real-robot demo
│   ├── mobilerobot.jpg    # robot photo (used in this README)
│   └── trajectory.jpg     # trajectory result from the experiment
├── install.sh             # spreads everything into the workspaces (+ optional build)
├── LICENSE                # GPL-3.0
├── .gitignore
└── README.md
```

## Prerequisites

- Ubuntu **22.04** (Jammy), x86\_64
- **ROS 2 Humble** (desktop install)
- `git`, `colcon`, `rosdep`, `cmake`, `build-essential`
- **Livox Mid-360** connected by Ethernet on the `192.168.1` subnet, powered on
- ≥ 8 GB free disk space, ≥ 8 GB RAM recommended for building FAST-LIO2

## Installation

### 0\. ROS 2 Humble + dependencies

```
sudo apt update
sudo apt install -y libeigen3-dev libpcl-dev libboost-all-dev \
                    python3-colcon-common-extensions python3-rosdep \
                    build-essential cmake git rsync
```

A `rosdep` database update (and `rosdep update` once) is used later — run them if
you have not yet:

```
sudo rosdep init && rosdep update   # skip sudo rosdep init if already done
```

> PCL and Eigen come straight from `apt`; the default versions on Jammy are
> sufficient for FAST-LIO2.

### 1\. Livox SDK2

```
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ~/Livox-SDK2
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 2\. Livox ROS 2 driver

Create the `fastlio_ws` workspace and clone the driver:

```
mkdir -p ~/fastlio_ws/src
cd ~/fastlio_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

Configure the **Mid-360 network** in the driver config — you MUST set your PC's IP
as the host address and the LiDAR's IP as the device address:

```
nano ~/fastlio_ws/src/livox_ros_driver2/config/MID360_config.json
```

```
{
  "host_net_info": {
    "host_ip": "192.168.1.50",
    "host_port": 0
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.114",
      "pcap": "",
      "fov_flag": false
    }
  ]
}
```

> `host_ip` = the Ubuntu PC's IP on the `192.168.1` NIC.
> `lidar_configs[].ip` = the Mid-360's fixed IP.

Now build and install the driver:

```
cd ~/fastlio_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
cd ~/fastlio_ws
source install/setup.bash
```

Sanity check — the raw LiDAR + IMU topics should appear:

```
ros2 launch livox_ros_driver2 msg_MID360_launch.py   # no GUI (for FAST-LIO2)
# or, to visualize first:
ros2 launch livox_ros_driver2 rviz_MID360_launch.py  # RViz only
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

> Use `msg_MID360_launch.py` for FAST-LIO2 (it publishes the Livox custom message
> with `offset_time`); `rviz_MID360_launch.py` is mainly for a quick visual check.

### 3\. FAST-LIO2

Clone FAST-LIO2 (the ROS 2 port) and install its dependencies:

```
cd ~/fastlio_ws/src
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
cd ~/fastlio_ws
rosdep install --from-paths src --ignore-src -r -y

# (re)build the livox driver if needed
cd ~/fastlio_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
cd ~/fastlio_ws
source install/setup.bash
```

Verify the packages are visible and locate the config:

```
ros2 pkg list | grep livox
ros2 pkg list | grep fast_lio
find ~/fastlio_ws/src/FAST_LIO_ROS2 -iname "*mid360*" -o -iname "*.yaml"
```

Edit the Mid-360 config used by FAST-LIO2:

```
nano ~/fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml
```

Set at minimum:

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
preprocess:
  lidar_type: 1        # 1 = Livox serials (Mid-360)
```

Check that the LiDAR message actually carries `offset_time` (needed by the
software time sync in FAST-LIO2):

```
ros2 topic echo /livox/lidar --once
ros2 topic echo /livox/imu --once
```

### 4\. This repository → `~/fastlio_ws` + `~/ros2_ws`

Clone the repo anywhere and run **`install.sh`** — it places `fastlio2_pose` into
the FAST-LIO2 workspace, **replaces the stock `mid360.yaml` with our tuned config**,
and copies `rover_control` into `~/ros2_ws` (optionally building both):

```
git clone https://github.com/alfinjunaedy/megarover-v3-fastlio2.git
cd megarover-v3-fastlio2

./install.sh            # copy files only
# or
./install.sh --build    # copy + colcon build fastlio2_pose and rover_control
```

> Run this **after** FAST-LIO2 is installed — `install.sh` replaces the default
> library config with the tuning used on this robot.

| Repo path | Installed to |
| --- | --- |
| `fastlio_ws/src/fastlio2_pose/` | `~/fastlio_ws/src/fastlio2_pose/` |
| `fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml` | `~/fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml` |
| `ros2_ws/src/rover_control/` | `~/ros2_ws/src/rover_control/` |
| `imgs/` | _(stays in the repo; GitHub README only)_ |

Custom locations are supported via environment variables:

```
FASTLIO_WS=~/my_fastlio_ws ROS2_WS=~/my_ros2_ws ./install.sh --build
```

To build manually instead of using `--build`:

```
source /opt/ros/humble/setup.bash
cd ~/fastlio_ws && colcon build --packages-select fastlio2_pose --symlink-install
cd ~/ros2_ws    && colcon build --packages-select rover_control --symlink-install
```

### 5\. Environment setup

Add to `~/.bashrc` so every terminal has the full stack:

```
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source ~/uros_ws/install/local_setup.bash
source ~/ros2_ws/install/local_setup.bash
source ~/fastlio_ws/install/local_setup.bash
```

Then `source ~/.bashrc` (or open a new terminal).

## Configuration — Network & IPs

| Address | Value | Role |
| --- | --- | --- |
| PC (host) IP | `192.168.1.50` | `host_net_info` in `MID360_config.json` |
| Mid-360 (device) IP | `192.168.1.114` | `lidar_configs[].ip` in `MID360_config.json` |
| Subnet | `192.168.1.0/24` | both interfaces must be on this net |

Make sure the PC's Ethernet interface is set statically (e.g. 192.168.1.50/24)
and that no firewall blocks UDP on the ports the Mid-360 uses (default 55000–55003).

## Usage

### A. One-command bring-up (recommended)

Power on the Mid-360 and the MegaRover V3, connect the base's USB-serial cable
(`/dev/ttyUSB0`), then:

```
ros2 launch rover_control rover_bringup_launch.py
```

The launch file brings up the components in a staggered sequence:

| t (s) | Component |
| --: | --- |
| 0 | micro-ROS Agent (`serial --dev /dev/ttyUSB0 --baudrate 115200 -v4`) |
| 0 | Livox Mid-360 driver (`msg_MID360_launch.py`) |
| 5 | FAST-LIO2 (`mapping.launch.py config_file:=.../mid360.yaml rviz:=false`) |
| 10 | `fastlio2_pose` (`/Odometry` → `/fastlio2_pose`) |
| 12 | RViz2 (`rviz_config.rviz`) |
| 14 | `rover_controller` (odometry + planner) |

### B. Manual terminals (the build-it-yourself way)

**Terminal 1 — LiDAR driver**

```
source ~/fastlio_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**Terminal 2 — FAST-LIO2**

```
source ~/fastlio_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=/home/robot/fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml \
  rviz:=false
```

> Change `/home/robot/...` to `/home/your-username/...` if your home is different.

**Terminal 3 — pose bridge + rover**

```
source ~/fastlio_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run fastlio2_pose fastlio2_pose_node        # /Odometry -> /fastlio2_pose
ros2 run rover_control rover_controller           # odometry + planner
```

In RViz (or `rviz2`) watch `/fastlio2_pose`, `/rover_odo`, `/cloud_registered`
and `/Laser_map`. The planner scripted in `rover_controller.cpp` drives the base
using the FAST-LIO2 estimate as feedback.

## Topics

| Topic | Type | Meaning |
| --- | --- | --- |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO2 LiDAR-inertial odometry (`camera_init` → `body`) |
| `/path` | `nav_msgs/Path` | FAST-LIO2 trajectory |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Registered (map-frame) point cloud |
| `/Laser_map` | `sensor_msgs/PointCloud2` | Global incremental map (ikd-Tree) |
| `/fastlio2_pose` | `geometry_msgs/PoseStamped` | **FAST-LIO2 pose re-published for the controller** |
| `/rover_odo` | `nav_msgs/Odometry` | Rover odometry (`rover_controller`) |
| `/rover_twist` | `geometry_msgs/Twist` | Velocity command to the base |
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Raw Mid-360 point cloud (`offset_time`) |
| `/livox/imu` | `sensor_msgs/Imu` | Mid-360 built-in IMU |
| `/tf`, `/tf_static` | transforms | odom / base / body frames |

## Rover Motion Programming

The rover's driving route is scripted directly in C++ — open
`~/ros2_ws/src/rover_control/src/rover_controller.cpp` and look for the
**`// Planner`** block. Commands execute **sequentially**, using the FAST-LIO2
pose (`/fastlio2_pose`) as feedback while the run is logged by the live
topics/RViz.

### Available motion commands

| Command | Arguments | Action |
| --- | --- | --- |
| `move_forward(dist, speed)` | `dist` \[m\], `speed` \[m/s\] | Drive straight forward |
| `move_backward(dist, speed)` | `dist` \[m\], `speed` \[m/s\] | Drive straight backward |
| `rotate_cw(deg, speed)` | `deg` \[°\], `speed` \[°/s\] | Rotate clockwise (turn right) in place |
| `rotate_ccw(deg, speed)` | `deg` \[°\], `speed` \[°/s\] | Rotate counter-clockwise (turn left) in place |
| `delay_seconds(s)` | `s` \[s\] | Pause between motions |

### Example — a surveyed path

```
// Planner *****************************************************
move_forward(1, 0.2);
delay_seconds(2);
rotate_cw(45);
move_forward(1, 0.2);
delay_seconds(2);
rotate_ccw(45);
// *************************************************************
```

### Rebuild after editing

```
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select rover_control
source install/setup.bash
```

Then relaunch — `rover_controller` starts 14 s into the bring-up sequence and
runs the planner immediately.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| No `/livox/lidar` topic | Check IPs in `MID360_config.json`, PC IP `192.168.1.50`, LiDAR `192.168.1.114`; ping the LiDAR (`ping 192.168.1.114`); allow UDP 55000–55003 |
| `/livox/lidar` echo has no `offset_time` | Use `msg_MID360_launch.py`, not `rviz_MID360_launch.py` |
| FAST-LIO2 exits immediately | Source the livox driver after build: `source ~/fastlio_ws/install/setup.bash`; confirm `lidar_type: 1` and topics match `/livox/lidar`, `/livox/imu` |
| `rosdep` can't find deps | `sudo rosdep update`, from `~/fastlio_ws`, `rosdep install --from-paths src --ignore-src -r -y` |
| `colcon build` killed on low RAM | `MAKEFLAGS="-j2" colcon build` |
| `serial: /dev/ttyUSB0: Permission denied` | `sudo usermod -aG dialout $USER`, re-login (or replug USB) |
| Agent starts but no rover topics | Power-cycle the base; keep the micro-ROS Agent start order (agent first in the launch file) |
| RViz shows no map/pose | `ros2 topic echo /fastlio2_pose --once` — if silent, check `/Odometry` and `/livox/lidar` streams |
| Mid-360 < v3.5.0 (no built-in IMU) | Use an external IMU and set `imu_topic` + `extrinsic_T/R` in `mid360.yaml`; enable `time_sync_en` only if external time sync is not possible |
| Map drifts over time | Ensure static PC IP and clean Ethernet; increase `det_range`/`filter_size_map` carefully; disable `extrinsic_est_en` once the extrinsic is calibrated |

## License

This repository is released under the **GNU GPL-3.0** — see
[LICENSE](https://github.com/alfinjunaedy/megarover-v3-fastlio2/blob/main/LICENSE).

Note on dependencies: **FAST-LIO / FAST_LIO_ROS2** (HKU-MARS / Ericsii) is
**GPL-2.0**, which is why this project (whose FAST-LIO2 workspace links against
it) is distributed under GPL. The `fastlio2_pose` bridge and the tuned
`mid360.yaml` in this repository are original. Vstone's `megarover3_ros2` and the
`livox_ros_driver2` are Apache-2.0 — you install those from their own
repositories.

## Acknowledgments

- [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) — FAST-LIO2 algorithm
- [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) — the ROS 2
  port this project is based on (`fast_lio`, `ros2 launch fast_lio mapping.launch.py`)
- [Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) and
  [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) —
  the Mid-360 SDK / ROS 2 driver
- [vstoneofficial/megarover3_ros2](https://github.com/vstoneofficial/megarover3_ros2) —
  MegaRover Ver.3.0 ROS 2 packages & micro-ROS setup (Vstone Co., Ltd.)
- [micro-ROS](https://micro.ros.org/) — micro\_ros\_setup / micro\_ros\_agent
- [alfinjunaedy/megarover-v3-orbslam3](https://github.com/alfinjunaedy/megarover-v3-orbslam3) —
  the previous ORB-SLAM3 (RGB-D) version of this project, whose controller is
  reused here

# 🛫 Clear-Run: Autonomous FOD Detection & Removal System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://python.org)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-4.4+-orange.svg)](https://ardupilot.org)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

> **A-FOD**: Autonomous Foreign Object Debris Detection & Removal for Airport Runways

Clear-Run is a heterogeneous multi-robot system that combines aerial detection (UAV) with ground-based retrieval (UGV) to autonomously identify and remove Foreign Object Debris from airport runways.

## 🎯 Project Overview

Foreign Object Debris (FOD) on runways poses a significant threat to aviation safety, causing millions in damages annually. Clear-Run addresses this with a **Coordinator-Worker** architecture:

- **Coordinator (UAV)**: Performs aerial surveillance using YOLOv11/v12 with RGB+IR imaging
- **Worker (UGV)**: Navigates to detected debris and collects it using an active brush-assisted scoop

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         CLEAR-RUN SYSTEM                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────┐         ┌─────────────────────┐           │
│  │      UAV (Drone)    │  WiFi   │      UGV (Rover)    │           │
│  │  ┌───────────────┐  │ ──────► │  ┌───────────────┐  │           │
│  │  │ Jetson Orin   │  │  ROS2   │  │ Jetson Nano   │  │           │
│  │  │    Nano       │  │ Topics  │  │               │  │           │
│  │  └───────┬───────┘  │         │  └───────┬───────┘  │           │
│  │          │ MAVROS   │         │          │ MAVROS   │           │
│  │  ┌───────▼───────┐  │         │  ┌───────▼───────┐  │           │
│  │  │ Pixhawk 2.4.8 │  │         │  │ Pixhawk 2.4.8 │  │           │
│  │  │  (ArduCopter) │  │         │  │  (ArduRover)  │  │           │
│  │  └───────────────┘  │         │  └───────────────┘  │           │
│  │                     │         │                     │           │
│  │  Sensors:           │         │  Sensors:           │           │
│  │  • 4K RGB Camera    │         │  • 2D LiDAR         │           │
│  │  • Thermal IR       │         │  • Depth Camera     │           │
│  │  • GPS Module       │         │  • Wheel Encoders   │           │
│  └─────────────────────┘         └─────────────────────┘           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## 🔄 Operational Flow

```
1. UAV SWEEP          2. DETECTION           3. VISUAL SERVO
   ┌────────┐            ┌────────┐            ┌────────┐
   │~~~~~~~~│            │  YOLO  │            │ CENTER │
   │  ████  │  ──────►   │ v11/12 │  ──────►   │  FOD   │
   │~~~~~~~~│            │ RGB+IR │            │ IN CAM │
   └────────┘            └────────┘            └────────┘
                                                    │
                                                    ▼
4. LOG GPS            5. DISPATCH UGV         6. COLLECT
   ┌────────┐            ┌────────┐            ┌────────┐
   │  LAT:  │            │ Nav2   │            │ BRUSH  │
   │  LON:  │  ◄──────   │ PATH   │  ──────►   │ SCOOP  │
   │  ALT:  │            │ PLAN   │            │ SWEEP  │
   └────────┘            └────────┘            └────────┘
```

## 📁 Repository Structure

```
Clear-Run/
├── README.md                    # This file
├── requirements.txt             # Python dependencies
├── docker/                      # Docker configurations
│   ├── Dockerfile.uav
│   ├── Dockerfile.ugv
│   └── docker-compose.yml
├── docs/                        # Documentation
│   ├── architecture.md
│   ├── hardware_setup.md
│   └── troubleshooting.md
├── src/                         # ROS 2 Packages
│   ├── clearrun_msgs/           # Custom message definitions
│   │   ├── msg/
│   │   │   └── FodLocation.msg
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   ├── clearrun_uav/            # UAV detection & control
│   │   ├── clearrun_uav/
│   │   │   ├── detection_node.py
│   │   │   ├── visual_servo.py
│   │   │   └── mavros_interface.py
│   │   ├── launch/
│   │   ├── config/
│   │   ├── package.xml
│   │   └── setup.py
│   ├── clearrun_ugv/            # UGV navigation & retrieval
│   │   ├── clearrun_ugv/
│   │   │   ├── fod_retriever.py
│   │   │   ├── scoop_controller.py
│   │   │   └── navigation_client.py
│   │   ├── launch/
│   │   ├── config/              # Nav2 parameters
│   │   ├── package.xml
│   │   └── setup.py
│   └── clearrun_bringup/        # System launch files
│       ├── launch/
│       ├── config/
│       ├── package.xml
│       └── setup.py
├── models/                      # YOLO weights & configs
│   ├── yolov11/
│   ├── yolov12/
│   └── README.md
├── simulation/                  # Gazebo & SITL
│   ├── worlds/
│   ├── models/
│   ├── launch/
│   └── sitl_configs/
└── scripts/                     # Utility scripts
    ├── setup_env.sh
    ├── calibrate_camera.py
    └── test_connection.py
```

## 🛠️ Hardware Requirements

### UAV (Coordinator)
| Component | Specification |
|-----------|---------------|
| Compute | NVIDIA Jetson Orin Nano (8GB) |
| Flight Controller | Pixhawk 2.4.8 |
| Firmware | ArduCopter 4.4+ |
| Camera | 4K RGB (Sony IMX477 or similar) |
| Thermal | FLIR Lepton 3.5 |
| GPS | u-blox M8N with compass |
| Frame | 450mm+ quadcopter |

### UGV (Worker)
| Component | Specification |
|-----------|---------------|
| Compute | NVIDIA Jetson Nano (4GB) |
| Controller | Pixhawk 2.4.8 |
| Firmware | ArduRover 4.4+ |
| LiDAR | RPLidar A2/A3 |
| Depth | Intel RealSense D435 |
| Chassis | Custom 4WD rover |
| Scoop | Active brush-assisted mechanism |

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10+
- NVIDIA CUDA 11.8+ (for Jetson)

### Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/Clear-Run.git
cd Clear-Run

# Install Python dependencies
pip install -r requirements.txt

# Build ROS 2 workspace
cd src
colcon build --symlink-install
source install/setup.bash
```

### Simulation (Gazebo + SITL)

```bash
# Terminal 1: Launch ArduPilot SITL for UAV
sim_vehicle.py -v ArduCopter --console --map

# Terminal 2: Launch ArduPilot SITL for UGV
sim_vehicle.py -v Rover -I1 --console

# Terminal 3: Launch Gazebo simulation
ros2 launch clearrun_bringup simulation.launch.py

# Terminal 4: Run the full system
ros2 launch clearrun_bringup clearrun_full.launch.py
```

## 📡 ROS 2 Topics

### UAV Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/uav/camera/image_raw` | sensor_msgs/Image | Raw camera feed |
| `/uav/camera/thermal` | sensor_msgs/Image | Thermal image |
| `/uav/detection/fod` | clearrun_msgs/FodLocation | Detected FOD location |
| `/uav/mavros/state` | mavros_msgs/State | Flight controller state |

### UGV Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/ugv/fod_target` | clearrun_msgs/FodLocation | Target FOD to collect |
| `/ugv/scan` | sensor_msgs/LaserScan | LiDAR scan data |
| `/ugv/scoop/status` | std_msgs/Bool | Scoop mechanism status |
| `/ugv/nav2/goal` | geometry_msgs/PoseStamped | Navigation goal |

## 🔧 Configuration

### Visual Servo Parameters
```yaml
# config/visual_servo.yaml
visual_servo:
  pid:
    kp: 0.5
    ki: 0.01
    kd: 0.1
  centering_threshold: 20  # pixels
  altitude_hold: 15.0      # meters
```

### Nav2 Parameters
```yaml
# config/nav2_params.yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
```

## 👥 Team

- **Muhammad Hanzalah Javed** - Avionics Engineering, CAE
- **Aneeq** - Avionics Engineering, CAE

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ArduPilot Community
- ROS 2 Community
- Ultralytics (YOLO)
- FOD Detection Research Community

---

<p align="center">
  <b>Clear-Run</b> - Making Runways Safer, One Debris at a Time 🛫
</p>

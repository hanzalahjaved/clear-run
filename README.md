# Clear-Run: Autonomous FOD Detection & Removal System
### 🚀 Final Year Project | CAE Avionics Engineering

![Build Status](https://img.shields.io/badge/ROS2-Humble-green)
![Python](https://img.shields.io/badge/Python-3.10-blue)
![AI Model](https://img.shields.io/badge/YOLO-v12-orange)

**Clear-Run** is a heterogeneous multi-agent robotic system designed to automate Foreign Object Debris (FOD) removal on airport runways. It closes the critical latency gap between detection and clearance by integrating a UAV (Detector) and a UGV (Retriever) into a unified "Coordinator-Worker" workflow.

---

## 🏗 System Architecture

The system operates in two distinct phases:

### Phase I: Detection (The Coordinator)
* **Platform:** Quadcopter UAV with Pixhawk 2.4.8 & Jetson Orin Nano.
* **Vision:** Custom-trained **YOLOv12** model (Hybrid RGB/IR dataset) for high-accuracy small object detection.
* **Geolocation:** Real-time transformation of 2D bounding boxes to 3D GPS coordinates (Lat/Lon) using flat-earth projection logic.

### Phase II: Removal (The Worker)
* **Platform:** UGV Rover with Jetson Nano.
* **Navigation:** ROS 2 Navigation Stack (Nav2) for path planning.
* **Actuation:** A novel **Hybrid Active Scoop** mechanism that combines mechanical intake with vacuum suction to retrieve debris of varying densities.

---

## 📂 Directory Structure

| Folder | Description |
| :--- | :--- |
| `ai_development/` | YOLOv12 training notebooks, weights (`.pt`), and dataset tools. |
| `src/uav_vision/` | ROS 2 nodes for on-board inference and coordinate transformation. |
| `src/ugv_control/` | ROS 2 navigation stack and scoop actuation drivers. |
| `src/clear_run_interfaces/` | Custom ROS 2 Actions (`RetrieveFod.action`) and Messages. |
| `src/simulation/` | Gazebo worlds (runway environment) and SDF models. |
| `docs/` | Wiring diagrams, CAD files, and user manuals. |

---

## 📦 Hardware Stack

| Component | UAV (Drone) | UGV (Rover) |
| :--- | :--- | :--- |
| **Flight/Motion Controller** | Pixhawk 2.4.8 (ArduPilot) | Arduino / ESP32 Base Controller |
| **Onboard Computer** | Jetson Orin Nano (AI Processing) | Jetson Nano / Raspberry Pi 4 |
| **Sensors** | 4K RGB Camera + Thermal IR | Lidar + Depth Camera |
| **Communication** | MAVLink (Telemetry) | ROS 2 / Micro-ROS (WiFi) |

---

## 🔧 Installation & Usage

### 1. Prerequisites
* Ubuntu 22.04 LTS
* ROS 2 Humble Hawksbill
* Python 3.10+

### 2. Setup
```bash
# Clone the repository
git clone [https://github.com/hanzalahjaved/clear-run.git](https://github.com/hanzalahjaved/clear-run.git)
cd clear-run

# Install Python dependencies
pip install -r requirements.txt

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
source install/setup.bash

# Clear-Run: Autonomous FOD Detection & Removal System
### 🚀 Final Year Project | CAE Avionics Engineering

**Clear-Run** is a multi-agent robotic system designed to automate Foreign Object Debris (FOD) removal. It utilizes a UAV for wide-area detection and a UGV for physical retrieval.

## 🏗 System Architecture

### Phase I: Detection (UAV)
* **Hardware:** Pixhawk 2.4.8 + Jetson Orin Nano.
* **Logic:** YOLOv12 detection triggers a **Visual Servoing** routine. The UAV centers itself directly over the FOD to capture high-accuracy GPS coordinates before transmission.

### Phase II: Removal (UGV)
* **Hardware:** Custom Chassis + Jetson Nano + Pixhawk (ArduRover).
* **Mechanism:** **Active Brush-Assisted Scoop**. 
    * Inspired by the *FODHippo* design.
    * Uses **active inward-rotating brushes** at the intake to sweep debris into the bin, ensuring even flat objects are captured.

## 📂 Directory Structure
* `ai_development/`: YOLOv12 weights and training data.
* `uav_control/`: Visual servoing and MAVROS scripts.
* `ugv_control/`: Nav2 configuration and scoop drivers.
* `simulation/`: Gazebo runway worlds and robot models.

## 📦 Installation
1. `git clone https://github.com/hanzalahjaved/clear-run.git`
2. `pip install -r requirements.txt`
3. `colcon build --symlink-install`

## 👥 Authors
* **Muhammad Hanzalah Javed**
* **Muhammad Aneeq**

#!/bin/bash

# 1. Create the high-level directories
mkdir -p ai_development/weights
mkdir -p ai_development/notebooks
mkdir -p ai_development/datasets

mkdir -p docs/hardware
mkdir -p docs/manuals
mkdir -p docs/images

# 2. Create the ROS 2 Workspace structure (src)
mkdir -p src/clear_run_interfaces/action
mkdir -p src/clear_run_interfaces/msg
mkdir -p src/clear_run_interfaces/srv

mkdir -p src/uav_vision/launch
mkdir -p src/uav_vision/scripts
mkdir -p src/uav_vision/config

mkdir -p src/ugv_control/launch
mkdir -p src/ugv_control/scripts
mkdir -p src/ugv_control/config

mkdir -p src/simulation/worlds
mkdir -p src/simulation/models

# 3. Create placeholder files so Git tracks the folders
touch src/clear_run_interfaces/action/RetrieveFod.action
touch src/clear_run_interfaces/msg/FodLocation.msg

touch src/uav_vision/scripts/detector_node.py
touch src/uav_vision/scripts/geo_tagger.py
touch src/uav_vision/config/camera_params.yaml

touch src/ugv_control/scripts/mission_controller.py
touch src/ugv_control/scripts/scoop_driver.py

touch ai_development/weights/.gitkeep
touch docs/hardware/wiring_diagram.pdf
touch requirements.txt

# 4. Make Python scripts executable
chmod +x src/uav_vision/scripts/*.py
chmod +x src/ugv_control/scripts/*.py

echo "✅ Clear-Run Project Structure Created Successfully!"

# =============================================================================
# Clear-Run Simulation
# =============================================================================

This directory contains simulation resources for testing the Clear-Run system
before deployment on actual hardware.

## Directory Structure

```
simulation/
├── worlds/              # Gazebo world files
│   └── runway.world     # Main runway environment with FOD objects
├── models/              # Custom Gazebo models
├── launch/              # Simulation-specific launch files
└── sitl_configs/        # ArduPilot SITL parameter files
    ├── sitl_uav.parm    # UAV (ArduCopter) configuration
    └── sitl_ugv.parm    # UGV (ArduRover) configuration
```

## Quick Start

### 1. Start ArduPilot SITL for UAV

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map -I 0
```

In the MAVProxy console, load parameters:
```
param load /path/to/Clear-Run/simulation/sitl_configs/sitl_uav.parm
```

### 2. Start ArduPilot SITL for UGV

In a new terminal:
```bash
cd ~/ardupilot/Rover
sim_vehicle.py -v Rover --console -I 1
```

Load parameters:
```
param load /path/to/Clear-Run/simulation/sitl_configs/sitl_ugv.parm
```

### 3. Launch Gazebo World (Gazebo Harmonic)

```bash
# Using the new Gazebo Sim (Harmonic)
gz sim /home/gondal/Clear-Run/simulation/worlds/runway.sdf

# Or with verbose output
gz sim -v 4 /home/gondal/Clear-Run/simulation/worlds/runway.sdf
```

### 4. Launch Clear-Run System

```bash
ros2 launch clearrun_bringup simulation.launch.py
```

## Gazebo World Features

The `runway.world` includes:
- 100m x 30m runway surface
- Center line and edge markings
- Sample FOD objects (bolts, washers, fragments, wire)
- Base station marker for home position

## Testing Scenarios

### Detection Test
1. Launch the system in simulation
2. Command UAV to fly survey pattern
3. Verify FOD detections appear in RViz

### Navigation Test
1. Publish a manual FodLocation message
2. Verify UGV navigates to target
3. Check scoop collection sequence

### Full System Test
1. Run complete mission from UAV survey to UGV collection
2. Verify inter-robot communication
3. Check system recovery from errors

## Tips

- Use `ros2 topic echo` to monitor message flow (ROS 2, not ROS 1)
- RViz2 shows real-time visualization
- Gazebo physics can be paused with spacebar or GUI controls
- Use `gz sim -r` to start simulation running immediately
- Use `gz topic -l` to list Gazebo topics
- Increase `real_time_factor` in world file for faster simulation

## Gazebo Harmonic Notes

This project uses **Gazebo Harmonic** (gz-sim 8.x), not Gazebo Classic.

| Old Command | New Command |
|-------------|-------------|
| `gazebo` | `gz sim` |
| `gzserver` | `gz sim -s` |
| `gzclient` | `gz sim -g` |
| `rostopic` | `gz topic` |

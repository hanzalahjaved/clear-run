#!/bin/bash
# =============================================================================
# Clear-Run Full Simulation Launcher
# =============================================================================
# This script launches the complete Clear-Run simulation environment:
#   1. Gazebo Harmonic with runway world
#   2. ArduPilot SITL for UAV (ArduCopter)
#   3. ArduPilot SITL for UGV (Rover)
#   4. MAVROS bridges
#   5. Clear-Run ROS 2 nodes
#
# Usage: ./run_simulation.sh [options]
#   --no-gazebo     Skip Gazebo launch (if already running)
#   --no-ardupilot  Skip ArduPilot SITL launch
#   --no-ros        Skip ROS 2 node launch (only SITL + Gazebo)
#   --uav-only      Launch only UAV components
#   --ugv-only      Launch only UGV components
#   --help          Show this help message
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ARDUPILOT_DIR="${HOME}/ardupilot"
SITL_UAV_PARM="${PROJECT_ROOT}/simulation/sitl_configs/sitl_uav.parm"
SITL_UGV_PARM="${PROJECT_ROOT}/simulation/sitl_configs/sitl_ugv.parm"
GAZEBO_WORLD="${PROJECT_ROOT}/simulation/worlds/runway.sdf"

# Options
LAUNCH_GAZEBO=true
LAUNCH_ARDUPILOT=true
LAUNCH_ROS=true
LAUNCH_UAV=true
LAUNCH_UGV=true

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-gazebo)
            LAUNCH_GAZEBO=false
            shift
            ;;
        --no-ardupilot)
            LAUNCH_ARDUPILOT=false
            shift
            ;;
        --no-ros)
            LAUNCH_ROS=false
            shift
            ;;
        --uav-only)
            LAUNCH_UGV=false
            shift
            ;;
        --ugv-only)
            LAUNCH_UAV=false
            shift
            ;;
        --help|-h)
            echo "Clear-Run Simulation Launcher"
            echo ""
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --no-gazebo     Skip Gazebo launch"
            echo "  --no-ardupilot  Skip ArduPilot SITL launch"
            echo "  --no-ros        Skip ROS 2 nodes (only SITL + Gazebo)"
            echo "  --uav-only      Launch only UAV components"
            echo "  --ugv-only      Launch only UGV components"
            echo "  --help          Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# Store PIDs for cleanup
PIDS=()

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down simulation...${NC}"
    
    # Kill all child processes
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Killing process $pid"
            kill -SIGTERM "$pid" 2>/dev/null || true
        fi
    done
    
    # Kill any remaining SITL processes
    pkill -f "arducopter" 2>/dev/null || true
    pkill -f "ardurover" 2>/dev/null || true
    pkill -f "sim_vehicle.py" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    
    echo -e "${GREEN}Simulation stopped.${NC}"
    exit 0
}

# Set up trap for cleanup
trap cleanup SIGINT SIGTERM

# Print banner
echo -e "${CYAN}"
echo "============================================================================="
echo "   _____ _                        ____              "
echo "  / ____| |                      |  _ \             "
echo " | |    | | ___  __ _ _ __ ______| |_) |_   _ _ __  "
echo " | |    | |/ _ \/ _\` | '__|______|  _ <| | | | '_ \ "
echo " | |____| |  __/ (_| | |         | |_) | |_| | | | |"
echo "  \_____|_|\___|\__,_|_|         |____/ \__,_|_| |_|"
echo ""
echo "         Autonomous FOD Detection & Removal System"
echo "============================================================================="
echo -e "${NC}"

# Check prerequisites
echo -e "${BLUE}[1/5] Checking prerequisites...${NC}"

# Check ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo -e "  ${GREEN}✓${NC} ROS 2 Jazzy found"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "  ${GREEN}✓${NC} ROS 2 Humble found"
else
    echo -e "  ${RED}✗${NC} ROS 2 not found! Please install ROS 2."
    exit 1
fi

# Check workspace build
if [ -f "${PROJECT_ROOT}/install/setup.bash" ]; then
    source "${PROJECT_ROOT}/install/setup.bash"
    echo -e "  ${GREEN}✓${NC} Clear-Run workspace built"
else
    echo -e "  ${YELLOW}!${NC} Workspace not built. Building now..."
    cd "$PROJECT_ROOT"
    colcon build --symlink-install
    source "${PROJECT_ROOT}/install/setup.bash"
    echo -e "  ${GREEN}✓${NC} Workspace built successfully"
fi

# Check Gazebo
if command -v gz &> /dev/null; then
    echo -e "  ${GREEN}✓${NC} Gazebo Harmonic found"
else
    echo -e "  ${RED}✗${NC} Gazebo not found! Install with: sudo apt install gz-harmonic"
    exit 1
fi

# Check ArduPilot
if [ -d "$ARDUPILOT_DIR" ]; then
    echo -e "  ${GREEN}✓${NC} ArduPilot found at $ARDUPILOT_DIR"
else
    echo -e "  ${YELLOW}!${NC} ArduPilot not found at $ARDUPILOT_DIR"
    if [ "$LAUNCH_ARDUPILOT" = true ]; then
        echo -e "  ${RED}✗${NC} Cannot launch SITL without ArduPilot. Use --no-ardupilot to skip."
        exit 1
    fi
fi

# Check for ardupilot venv
ARDUPILOT_VENV="${HOME}/venv-ardupilot"
if [ -d "$ARDUPILOT_VENV" ]; then
    echo -e "  ${GREEN}✓${NC} ArduPilot venv found"
fi

echo ""

# Launch Gazebo
if [ "$LAUNCH_GAZEBO" = true ]; then
    echo -e "${BLUE}[2/5] Launching Gazebo Harmonic...${NC}"
    
    if [ ! -f "$GAZEBO_WORLD" ]; then
        echo -e "  ${RED}✗${NC} World file not found: $GAZEBO_WORLD"
        exit 1
    fi
    
    gz sim "$GAZEBO_WORLD" &
    PIDS+=($!)
    echo -e "  ${GREEN}✓${NC} Gazebo launched (PID: ${PIDS[-1]})"
    sleep 3
else
    echo -e "${BLUE}[2/5] Skipping Gazebo launch${NC}"
fi

echo ""

# Launch ArduPilot SITL
if [ "$LAUNCH_ARDUPILOT" = true ]; then
    echo -e "${BLUE}[3/5] Launching ArduPilot SITL...${NC}"
    
    # Activate ArduPilot venv if exists
    if [ -f "${ARDUPILOT_VENV}/bin/activate" ]; then
        source "${ARDUPILOT_VENV}/bin/activate"
    fi
    
    # Launch UAV SITL
    if [ "$LAUNCH_UAV" = true ]; then
        echo -e "  Starting ArduCopter SITL (Instance 0)..."
        cd "${ARDUPILOT_DIR}/ArduCopter"
        
        # Start SITL - native mode without Gazebo plugin
        sim_vehicle.py -v ArduCopter -I 0 --no-mavproxy -w &
        PIDS+=($!)
        echo -e "  ${GREEN}✓${NC} ArduCopter SITL started (PID: ${PIDS[-1]})"
        sleep 8
    fi
    
    # Launch UGV SITL
    if [ "$LAUNCH_UGV" = true ]; then
        echo -e "  Starting Rover SITL (Instance 1)..."
        cd "${ARDUPILOT_DIR}/Rover"
        
        sim_vehicle.py -v Rover -I 1 --no-mavproxy -w &
        PIDS+=($!)
        echo -e "  ${GREEN}✓${NC} Rover SITL started (PID: ${PIDS[-1]})"
        sleep 5
    fi
    
    cd "$PROJECT_ROOT"
else
    echo -e "${BLUE}[3/5] Skipping ArduPilot SITL launch${NC}"
fi

echo ""

# Launch MAVROS
echo -e "${BLUE}[4/5] Launching MAVROS bridges...${NC}"

# ArduPilot SITL ports:
# Instance 0 (UAV): TCP 5760, UDP out 14550
# Instance 1 (UGV): TCP 5770, UDP out 14560

if [ "$LAUNCH_UAV" = true ]; then
    echo -e "  Starting MAVROS for UAV (connecting to tcp:127.0.0.1:5760)..."
    ros2 run mavros mavros_node --ros-args \
        -p fcu_url:=tcp://127.0.0.1:5760 \
        -p tgt_system:=1 \
        -p tgt_component:=1 \
        -r __ns:=/uav &
    PIDS+=($!)
    echo -e "  ${GREEN}✓${NC} MAVROS UAV started (PID: ${PIDS[-1]})"
fi

if [ "$LAUNCH_UGV" = true ]; then
    echo -e "  Starting MAVROS for UGV (connecting to tcp:127.0.0.1:5770)..."
    ros2 run mavros mavros_node --ros-args \
        -p fcu_url:=tcp://127.0.0.1:5770 \
        -p tgt_system:=1 \
        -p tgt_component:=1 \
        -r __ns:=/ugv &
    PIDS+=($!)
    echo -e "  ${GREEN}✓${NC} MAVROS UGV started (PID: ${PIDS[-1]})"
fi

sleep 2
echo ""

# Launch Clear-Run nodes
if [ "$LAUNCH_ROS" = true ]; then
    echo -e "${BLUE}[5/5] Launching Clear-Run ROS 2 nodes...${NC}"

    if [ "$LAUNCH_UAV" = true ]; then
        echo -e "  Launching UAV detection nodes..."
        ros2 run clearrun_uav detection_node --ros-args -r __ns:=/uav &
        PIDS+=($!)
        ros2 run clearrun_uav visual_servo --ros-args -r __ns:=/uav &
        PIDS+=($!)
    fi
    
    if [ "$LAUNCH_UGV" = true ]; then
        echo -e "  Launching UGV retrieval nodes..."
        ros2 run clearrun_ugv fod_retriever --ros-args -r __ns:=/ugv &
        PIDS+=($!)
        ros2 run clearrun_ugv scoop_controller --ros-args -r __ns:=/ugv &
        PIDS+=($!)
    fi

    echo -e "  ${GREEN}✓${NC} Clear-Run nodes launched"
else
    echo -e "${BLUE}[5/5] Skipping ROS 2 node launch${NC}"
fi

echo ""
echo -e "${GREEN}=============================================================================${NC}"
echo -e "${GREEN}  Simulation is running!${NC}"
echo -e "${GREEN}=============================================================================${NC}"
echo ""
echo -e "  ${CYAN}Gazebo:${NC}     Runway world with FOD objects"
if [ "$LAUNCH_UAV" = true ]; then
    echo -e "  ${CYAN}UAV SITL:${NC}   tcp://127.0.0.1:5760 (ArduCopter, Instance 0)"
    echo -e "  ${CYAN}UAV MAVROS:${NC} /uav namespace"
fi
if [ "$LAUNCH_UGV" = true ]; then
    echo -e "  ${CYAN}UGV SITL:${NC}   tcp://127.0.0.1:5770 (Rover, Instance 1)"
    echo -e "  ${CYAN}UGV MAVROS:${NC} /ugv namespace"
fi
echo ""
echo -e "  ${YELLOW}Useful commands:${NC}"
echo -e "    ros2 topic list                     # List all topics"
echo -e "    ros2 topic echo /fod_detections     # View FOD detections"
echo -e "    ros2 topic echo /uav/mavros/state   # UAV state"
echo -e "    ros2 topic echo /ugv/mavros/state   # UGV state"
echo -e "    rviz2                               # Launch RViz2"
echo ""
echo -e "  ${RED}Press Ctrl+C to stop the simulation${NC}"
echo ""

# Wait for all processes
wait

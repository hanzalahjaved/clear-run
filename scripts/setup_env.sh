#!/bin/bash
# =============================================================================
# Clear-Run Environment Setup Script
# =============================================================================
# This script sets up the development environment for Clear-Run
# Run with: source scripts/setup_env.sh
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================="
echo "     Clear-Run Environment Setup"
echo "============================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print status
print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# Check ROS 2 installation
echo ""
echo "Checking ROS 2 installation..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    print_status "ROS 2 Humble found and sourced"
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash
    print_status "ROS 2 Iron found and sourced"
else
    print_error "ROS 2 not found! Please install ROS 2 Humble or Iron"
    return 1
fi

# Check for required ROS 2 packages
echo ""
echo "Checking required ROS 2 packages..."
REQUIRED_PKGS=("mavros" "nav2_bringup" "gazebo_ros_pkgs")
MISSING_PKGS=()

for pkg in "${REQUIRED_PKGS[@]}"; do
    if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
        print_status "$pkg installed"
    else
        print_warning "$pkg not found"
        MISSING_PKGS+=("$pkg")
    fi
done

if [ ${#MISSING_PKGS[@]} -gt 0 ]; then
    echo ""
    print_warning "Missing packages. Install with:"
    echo "  sudo apt install ros-\$ROS_DISTRO-${MISSING_PKGS[*]// / ros-\$ROS_DISTRO-}"
fi

# Check Python version
echo ""
echo "Checking Python..."
PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2)
if [[ "$PYTHON_VERSION" > "3.10" ]] || [[ "$PYTHON_VERSION" == "3.10"* ]]; then
    print_status "Python $PYTHON_VERSION found"
else
    print_warning "Python 3.10+ recommended. Found: $PYTHON_VERSION"
fi

# Check CUDA (for Jetson/GPU)
echo ""
echo "Checking CUDA..."
if command -v nvcc &> /dev/null; then
    CUDA_VERSION=$(nvcc --version | grep "release" | awk '{print $6}' | cut -d',' -f1)
    print_status "CUDA $CUDA_VERSION found"
else
    print_warning "CUDA not found. GPU acceleration will not be available."
fi

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
if [ -f "$PROJECT_ROOT/requirements.txt" ]; then
    pip install -r "$PROJECT_ROOT/requirements.txt" --quiet 2>/dev/null || {
        print_warning "Some packages failed to install. Run manually:"
        echo "  pip install -r requirements.txt"
    }
    print_status "Python dependencies processed"
else
    print_warning "requirements.txt not found"
fi

# Build ROS 2 workspace
echo ""
echo "Building ROS 2 workspace..."
cd "$PROJECT_ROOT"

if [ -d "src" ]; then
    # Check if already built
    if [ -d "install" ]; then
        print_status "Workspace already built. To rebuild, run:"
        echo "  cd $PROJECT_ROOT && colcon build --symlink-install"
    else
        echo "Building workspace (this may take a few minutes)..."
        colcon build --symlink-install 2>&1 | tail -5
        print_status "Workspace built"
    fi
    
    # Source the workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        print_status "Workspace sourced"
    fi
else
    print_warning "src directory not found"
fi

# Set environment variables
echo ""
echo "Setting environment variables..."
export CLEARRUN_ROOT="$PROJECT_ROOT"
export GAZEBO_MODEL_PATH="${PROJECT_ROOT}/simulation/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${PROJECT_ROOT}/simulation/worlds:${GAZEBO_RESOURCE_PATH:-}"

print_status "CLEARRUN_ROOT=$CLEARRUN_ROOT"
print_status "GAZEBO paths updated"

# Check ArduPilot SITL
echo ""
echo "Checking ArduPilot SITL..."
if command -v sim_vehicle.py &> /dev/null; then
    print_status "ArduPilot SITL found"
else
    print_warning "ArduPilot SITL not found in PATH"
    echo "  Install from: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html"
fi

# Summary
echo ""
echo "============================================="
echo "     Setup Complete!"
echo "============================================="
echo ""
echo "Quick start commands:"
echo "  - Build:      cd $PROJECT_ROOT && colcon build --symlink-install"
echo "  - Simulate:   ros2 launch clearrun_bringup simulation.launch.py"
echo "  - Full run:   ros2 launch clearrun_bringup clearrun_full.launch.py"
echo ""
echo "For more information, see README.md"
echo ""

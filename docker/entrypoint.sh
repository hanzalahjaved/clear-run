#!/bin/bash
# =============================================================================
# Clear-Run Docker Entrypoint
# =============================================================================

set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f "/clearrun_ws/install/setup.bash" ]; then
    source /clearrun_ws/install/setup.bash
fi

# Execute the command
exec "$@"

#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Source your workspace (if built)
if [ -f "/workspace/autonomous_nav/install/setup.bash" ]; then
    source /workspace/autonomous_nav/install/setup.bash
fi

# Export important env vars
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=9
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Execute passed command
exec "$@"
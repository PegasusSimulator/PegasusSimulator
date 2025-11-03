#!/bin/bash -e

# Environment variables with defaults
USER_ID=${LOCAL_USER_ID:-1000}
GROUP_ID=${LOCAL_GROUP_ID:-1000}
USERNAME=ubuntu
ROS_DISTRO=${ROS_DISTRO:-humble}

# Isaac Sim specific environment variables
export ACCEPT_EULA=Y
export PRIVACY_CONSENT=N
export ISAACSIM_PATH=/isaac-sim
export ISAACSIM_PYTHON=/isaac-sim/python.sh
export ISAACSIM=/isaac-sim/isaac-sim.sh

# PX4 environment variables
export PX4_DIR=/home/ubuntu/PX4-Autopilot

# ROS2 configuration (fixed to Humble)
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/humble/lib"

# Source ROS2 environment
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "Sourcing ROS2 ${ROS_DISTRO} environment..."
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source workspace if it exists
if [ -f "$ROS_WS/install/setup.bash" ]; then
    echo "Sourcing workspace environment..."
    source "$ROS_WS/install/setup.bash"
fi

echo "Using RMW implementation: $RMW_IMPLEMENTATION"

# Change to PegasusSimulator directory
cd /home/ubuntu/PegasusSimulator

# Check and install Pegasus Simulator extension (only if not already installed)
PEGASUS_INSTALLED=$(${ISAACSIM_PYTHON} -m pip list 2>/dev/null | grep -c "pegasus-simulator" || echo "0")

if [ "$PEGASUS_INSTALLED" = "0" ]; then
    echo "Installing Pegasus Simulator extension (first time setup)..."
    cd /home/ubuntu/PegasusSimulator/extensions
    ${ISAACSIM_PYTHON} -m pip install --editable pegasus.simulator
    cd /home/ubuntu/PegasusSimulator
    echo "Pegasus Simulator extension installed."
else
    echo "Pegasus Simulator extension already installed"
fi

# Create symbolic link for Isaac Sim to find extension assets
rm -rf /root/.local/share/ov/data/exts/v2/pegasus.simulator-*
ln -sf /home/ubuntu/PegasusSimulator/extensions/pegasus.simulator /root/.local/share/ov/data/exts/v2/pegasus.simulator-4.5.0

# Auto run program
if [ "${AUTO_RUN}" = "true" ]; then
    echo "Launching Pegasus Simulator example..."
    cd /home/ubuntu/PegasusSimulator/examples
    ${ISAACSIM_PYTHON} 1_px4_single_vehicle.py
fi

# If no command is provided, start an idle loop to keep the container alive
if [ $# -eq 0 ]; then
    echo "Container ready. Use './isaac-launch.sh enter' to access the shell."
    while true; do sleep 60; done
else
    echo "Executing command as $USERNAME: $@"
    # exec gosu $USERNAME "$@"
    exec "$@"
fi

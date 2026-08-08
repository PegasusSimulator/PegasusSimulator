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
echo "Checking Pegasus Simulator installation..."
PEGASUS_INSTALLED=$(${ISAACSIM_PYTHON} -m pip list 2>/dev/null | grep -c "pegasus-simulator" || true)

if [ -z "$PEGASUS_INSTALLED" ] || [ "$PEGASUS_INSTALLED" = "0" ]; then
    echo "Installing Pegasus Simulator extension (first time setup)..."
    cd /home/ubuntu/PegasusSimulator/extensions
    
    # Remove old egg-info if exists
    rm -rf pegasus.simulator/pegasus_simulator.egg-info 2>/dev/null || true
    
    # Install the package
    ${ISAACSIM_PYTHON} -m pip install --editable pegasus.simulator || {
        echo "ERROR: Failed to install Pegasus Simulator!"
        exit 1
    }
    
    # Fix permissions for the egg-info directory
    chown -R ubuntu:ubuntu pegasus.simulator/pegasus_simulator.egg-info 2>/dev/null || true
    
    cd /home/ubuntu/PegasusSimulator
    echo "✓ Pegasus Simulator extension installed successfully."
else
    echo "✓ Pegasus Simulator extension already installed (found $PEGASUS_INSTALLED package(s))"
    # Ensure permissions are correct even if already installed
    chown -R ubuntu:ubuntu /home/ubuntu/PegasusSimulator/extensions/pegasus.simulator/pegasus_simulator.egg-info 2>/dev/null || true
fi

# Create necessary directories with proper permissions
echo "Creating necessary directories..."
mkdir -p /isaac-sim/kit/data/Kit/"Isaac-Sim Python"/4.5/pip3-envs/default
mkdir -p /home/ubuntu/.config/pulse
mkdir -p /home/ubuntu/Documents/Kit/apps/"Isaac-Sim Python"/scripts/new_stage
mkdir -p /home/ubuntu/Documents/Kit/shared/screenshots
mkdir -p /isaac-sim/kit/logs

# Create symbolic link for Isaac Sim to find extension assets
rm -rf /root/.local/share/ov/data/exts/v2/pegasus.simulator-*
ln -sf /home/ubuntu/PegasusSimulator/extensions/pegasus.simulator /root/.local/share/ov/data/exts/v2/pegasus.simulator-4.5.0

# Set proper ownership
chown -R ubuntu:ubuntu /home/ubuntu/.config 2>/dev/null || true
chown -R ubuntu:ubuntu /home/ubuntu/Documents 2>/dev/null || true

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


# ================= ISAAC SIM ================= #
# == Description: Isaac Sim environment setup = #
# =================== BEGIN =================== #

# Isaac Sim root directory
export ISAACSIM_PATH="/isaac-sim"
# Isaac Sim python executable
export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
# Isaac Sim app
export ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"

# Define an auxiliary function to launch Isaac Sim or run scripts with Isaac Sim's python
# This is done to avoid conflicts between ROS 2 and Isaac Sim's Python environment
isaac_run() {

    # ------------------
    # === VALIDATION ===
    # ------------------
    if [ ! -x "$ISAACSIM_PYTHON" ]; then
        echo "❌ IsaacSim python.sh not found at: $ISAACSIM_PYTHON"
        return 1
    fi
    if [ ! -x "$ISAACSIM" ]; then
        echo "❌ IsaacSim launcher not found at: $ISAACSIM"
        return 1
    fi

    # -------------------------
    # === CLEAN ENVIRONMENT ===
    # -------------------------
    # Unset ROS 2 environment variables to avoid conflicts with Isaac's Python 3.11
    unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH CMAKE_PREFIX_PATH

    # Remove ROS 2 paths from LD_LIBRARY_PATH if present
    local ros_paths=("/opt/ros/humble" "/opt/ros/jazzy" "/opt/ros/iron")
    for ros_path in "${ros_paths[@]}"; do
        export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "^${ros_path}" | paste -sd':' -)
    done

    # -------------------------
    # === ROS2 CONFIGURATION ===
    # -------------------------
    # Fixed to ROS2 Humble with FastRTPS (as per official documentation)
        export ROS_DISTRO=humble
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/humble/lib"

    # ---------------------
    # === RUN ISAAC SIM ===
    # ---------------------
    if [ $# -eq 0 ]; then
        # No args → Launch full Isaac Sim GUI
        echo "🧠 Launching Isaac Sim GUI..."
        "${ISAACSIM}"

    elif [[ "$1" == --* ]]; then
        # Arguments start with "--" → pass them to Isaac Sim executable
        echo "⚙️  Launching Isaac Sim with options: $*"
        "${ISAACSIM}" "$@"

    elif [ -f "$1" ]; then
        # First argument is a Python file → run with Isaac Sim's Python
        local SCRIPT_PATH="$1"
        shift
        echo "🚀 Running Python script with Isaac Sim: $SCRIPT_PATH"
        "${ISAACSIM_PYTHON}" "$SCRIPT_PATH" "$@"

    else
        # Unrecognized input
        echo "❌ Unknown argument or file not found: '$1'"
        echo "Usage:"
        echo "  isaac_run                 → launch GUI"
        echo "  isaac_run my_script.py    → run script with IsaacSim Python"
        echo "  isaac_run --headless ...  → launch IsaacSim with CLI flags"
        return 1
    fi
}

# Alias for convenience
alias isim='isaac_run'

# ==================== END ==================== #


# ==================== ROS ==================== #
# ==== Description: ROS environment setup ===== #
# =================== BEGIN =================== #

source_ros_environment() {
    if [ "$ROS_DISTRO" = "humble" ]; then
        # Custom Alias
        alias rosdep-check='rosdep install -i --from-path src --rosdistro humble -y'
        alias build='colcon build --symlink-install'

        # Source ROS environment
        source /opt/ros/humble/setup.bash
        source $ROS_WS/install/setup.bash

        # Source workspace environment
        # latest_setup_bash: Find the latest setup.bash over user's root directory based on the last modified time
        latest_setup_bash=$(find $(pwd) -type f -name "setup.bash" -wholename "*/install/setup.bash" -printf "%T@ %p\n" | sort -nr | awk '{print $2}' | head -n 1)
        if [ -n "$latest_setup_bash" ]; then
            source $latest_setup_bash
            # Source colcon environment
            source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
            source /usr/share/colcon_cd/function/colcon_cd.sh
            export _colcon_cd_root="${latest_setup_bash%/install/setup.bash}"
        else
            echo "No setup.bash file found for humble"
        fi
    elif [ "$ROS_DISTRO" = "noetic" ]; then
        source /opt/ros/noetic/setup.bash
        # Source workspace environment
        if [ -n "$ROS_WS" ]; then
            source $ROS_WS/devel/setup.bash
        else
            echo "ROS_WS variable is not set for noetic"
        fi
    else
        echo "Unsupported ROS_DISTRO: $ROS_DISTRO"
    fi
}

# ==================== END ==================== #



# ================= Functions ================= #
# Note: If you want to use the custom function, #
#        you need to uncomment the line below.  #
# =================== BEGIN =================== #

source_ros_environment

# PX4-Autopilot environment
export PX4_DIR=/home/ubuntu/PX4-Autopilot

# Aliases for Pegasus Simulator
alias pegasus-install='cd /home/ubuntu/PegasusSimulator/extensions && ${ISAACSIM_PYTHON} -m pip install --editable pegasus.simulator'
alias pegasus-examples='cd /home/ubuntu/PegasusSimulator/examples'
alias qgroundcontrol='/home/ubuntu/QGroundControl.AppImage'
alias qgc='qgroundcontrol'

# ==================== END ==================== #

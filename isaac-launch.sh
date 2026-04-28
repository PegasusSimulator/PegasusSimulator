#!/bin/bash

# Get the directory of the script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd $DIR

# Open Xhost for Isaac Sim GUI
xhost +local: > /dev/null 2>&1

cd ./docker

# Environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:=100}

# Create or overwrite the .env file with static settings
cat <<EOL > .env
# Docker Compose Settings
COMPOSE_PROJECT_NAME=pegasus-simulator
COMPOSE_BAKE=true

# ROS Settings
ROS_DISTRO=humble

# Dynamic UID and GID
# LOCAL_USER_ID=$(id -u)
# LOCAL_GROUP_ID=$(id -g)
LOCAL_USER_ID=1000
LOCAL_GROUP_ID=1000
EOL

# Ensure cleanup on script exit
cleanup() {
    if [[ "$ACTION" != "close" && "$ACTION" != "run" && "$ACTION" != "sim" ]]; then
        if docker ps --filter "name=pegasus-simulator" --filter "status=running" | grep -q "pegasus-simulator"; then
            echo "Container is still running. Use 'close' to stop it."
        fi
    elif [[ "$ACTION" == "close" ]]; then
        if docker ps --filter "name=pegasus-simulator" --filter "status=running" | grep -q "pegasus-simulator"; then
            docker compose down
        fi
    fi
    xhost -local: > /dev/null 2>&1
}
trap cleanup EXIT

# Argument handling
ACTION=${1:-}
CUSTOM_COMMAND=${2:-}

print_usage() {
    echo -e "\033[1;32m----- [ isaac-launch Usage ] ----------------\033[0m"
    echo -e "\033[1;32m|\033[0m   run             - Run Pegasus example (PX4 single vehicle)"
    echo -e "\033[1;32m|\033[0m   sim             - Launch Isaac Sim GUI"
    echo -e "\033[1;32m|\033[0m   qgc             - Launch QGroundControl"
    echo -e "\033[1;32m|\033[0m   enter           - Enter the container"
    echo -e "\033[1;32m|\033[0m   close           - Stop the container"
    echo -e "\033[1;32m|\033[0m   --command <cmd> - Run a custom command"
    echo -e "\033[1;32m---------------------------------------------\033[0m"
}

if [[ "$ACTION" == "--command" ]]; then
    if [[ -z "$CUSTOM_COMMAND" ]]; then
        echo "Error: No command specified for --command."
        exit 1
    fi
    if ! docker ps --filter "name=pegasus-simulator" --filter "status=running" | grep -q "pegasus-simulator"; then
        echo "Container 'pegasus-simulator' is not running. Starting it..."
        docker compose up -d
    fi
    docker compose exec pegasus-simulator bash -c "$CUSTOM_COMMAND"
    exit 0
fi

case $ACTION in
    run)
        export AUTO_BUILD=false AUTO_RUN=true
        echo "Running Pegasus Simulator example..."
        docker compose up
        docker compose down
        ;;
    sim)
        export AUTO_BUILD=false AUTO_RUN=false
        echo "Launching Isaac Sim GUI..."
        docker compose up -d
        docker compose exec \
            -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
            -e LD_LIBRARY_PATH="/opt/ros/humble/lib:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH}" \
            pegasus-simulator bash -c "/isaac-sim/isaac-sim.sh --enable isaacsim.ros2.bridge"
        echo "Isaac Sim process exited. Stopping the container..."
        docker compose down
        ;;
    qgc)
        export AUTO_BUILD=false AUTO_RUN=false
        echo "Launching QGroundControl..."
        if ! docker ps --filter "name=pegasus-simulator" --filter "status=running" | grep -q "pegasus-simulator"; then
            echo -e "Container 'pegasus-simulator' is not running.\nStarting container..."
            docker compose up -d
        fi
        docker exec -it --user ubuntu pegasus-simulator bash -c "/home/ubuntu/QGroundControl.AppImage" || echo "QGroundControl exited."
        ;;
    enter)
        export AUTO_BUILD=false AUTO_RUN=false
        echo "Entering the container..."
        if ! docker ps --filter "name=pegasus-simulator" --filter "status=running" | grep -q "pegasus-simulator"; then
            echo -e "Container 'pegasus-simulator' is not running."
            docker compose up -d
        fi
        docker exec -it --user ubuntu pegasus-simulator bash || echo "Exited container without shutting it down."
        ;;
    close)
        export AUTO_BUILD=false AUTO_RUN=false
        if docker ps --filter "name=pegasus-simulator" --filter "status=running" | grep -q "pegasus-simulator" || \
           docker ps --filter "name=pegasus-simulator" --filter "status=exited" | grep -q "pegasus-simulator"; then
            echo "Stopping the container..."
            docker compose down
        else
            echo "Container is already stopped."
        fi
        ;;
    *)
        print_usage
        exit 1
        ;;
esac

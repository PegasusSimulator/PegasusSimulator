## Docker Support for Pegasus Simulator

For a simplified setup experience, we provide Docker support that eliminates the need for manual Isaac Sim and PX4-Autopilot installation. This approach is particularly useful for users who want to get started quickly or avoid potential dependency conflicts.

### Prerequisites
- NVIDIA GPU and Docker support
- [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/install/)
- NVIDIA Container Runtime for GPU support
- X11 server for GUI support (usually pre-installed on Linux)

### Quick Start
Simply run the following command to launch Isaac Sim GUI - Docker will build automatically:
```bash
./isaac-launch.sh sim
```

### Docker Usage

The `isaac-launch.sh` script provides convenient commands for managing the Docker environment:

- `./isaac-launch.sh run` - Run Pegasus example (PX4 single vehicle)
- `./isaac-launch.sh sim` - Launch Isaac Sim GUI with ROS2 bridge enabled
- `./isaac-launch.sh qgc` - Launch QGroundControl
- `./isaac-launch.sh enter` - Enter the container for interactive development
- `./isaac-launch.sh close` - Stop and remove the container
- `./isaac-launch.sh --command <cmd>` - Execute a custom command in the container

**Note**: Pegasus Simulator extension is automatically installed on first container startup.

#### Run Example Simulation:
```bash
# Run the PX4 single vehicle example
./isaac-launch.sh run
```

#### Interactive Development:
```bash
# Start the container and enter for development
./isaac-launch.sh enter

# Inside the container, you can:
# 1. Navigate to examples directory
pegasus-examples

# 2. Run examples using Isaac Sim's Python
isaac_run 1_px4_single_vehicle.py

# 3. Launch Isaac Sim GUI
isaac_run

# 4. Launch QGroundControl (in a new terminal)
qgroundcontrol
# or simply
qgc
```

### Environment Configuration

The Docker setup includes:
- **Isaac Sim 4.5.0** (standalone installation)
- **PX4-Autopilot v1.14.3** (pre-compiled for SITL)
- **QGroundControl** (latest stable version, pre-installed as AppImage)
- **ROS2 Humble Desktop** with FastRTPS (default DDS middleware)
- **Pegasus Simulator** (mounted from host repository for live development)
- **GPU acceleration** (NVIDIA Container Toolkit)
- **Python 3.10** with Isaac Sim Python environment
- **Development tools**: tmux, vim, git, htop, tree, bash-completion

**Environment Variables:**
- `ROS_DOMAIN_ID=100` (default, configurable)
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ISAACSIM_PATH=/isaac-sim`
- `PX4_DIR=/home/ubuntu/PX4-Autopilot`

### Directory Structure

Inside the container:
- `/isaac-sim/` - Isaac Sim 4.5.0 installation
- `/home/ubuntu/PegasusSimulator/` - Pegasus Simulator repository (mounted from host)
- `/home/ubuntu/PX4-Autopilot/` - PX4-Autopilot v1.14.3 installation
- `/home/ubuntu/QGroundControl.AppImage` - QGroundControl application

**Mounted volumes from host:**
- `../` → `/home/ubuntu/PegasusSimulator` (repository root)
- `~/App/isaac-sim/cache/` → Isaac Sim cache directories
- `~/App/QGroundControl/` → QGroundControl configuration and data

### Using QGroundControl
QGroundControl is pre-installed for controlling PX4 vehicles:

1. Start a simulation with PX4:
```bash
./isaac-launch.sh run
# or inside container
isaac_run 1_px4_single_vehicle.py
```

2. In another terminal, launch QGroundControl:
```bash
./isaac-launch.sh enter
qgroundcontrol
```

3. QGroundControl will automatically connect to the simulated vehicle (MAVLink on UDP port 14550)

### Advanced Usage

#### Custom Commands

Execute any command in the container without entering interactive mode:

```bash
# Build ROS2 workspace
./isaac-launch.sh --command "cd /home/ubuntu/PegasusSimulator/ros2_ws && colcon build"
```

Note: The `.bashrc` is automatically sourced when using `--command`, so all aliases and functions (like `isaac_run`, `qgc`, etc.) are available.

#### Modify ROS Domain ID

Change the ROS_DOMAIN_ID before starting the container:

```bash
export ROS_DOMAIN_ID=50
./isaac-launch.sh sim
```

#### Persistence and Data

The following directories persist across container restarts:
- Isaac Sim cache and configuration (`~/App/isaac-sim/`)
- QGroundControl settings (`~/App/QGroundControl/`)
- Your code changes (repository is mounted from host)

### Notes
- The container uses Ubuntu 22.04 with ROS2 Humble
- The Pegasus Simulator repository is mounted from the host for easy development
- **Pegasus Simulator extension is auto-installed on first startup**
- PX4-Autopilot is pre-installed and configured for SITL
- QGroundControl is pre-configured with necessary dependencies
- Use `isaac_run` function to run scripts with proper environment isolation
- User is added to `dialout` group for serial port access

### References
- [Pegasus Simulator Installation Guide](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)
- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [PX4-Autopilot Documentation](https://docs.px4.io/)

*Docker integration developed by [SeanChangX](https://github.com/SeanChangX)*

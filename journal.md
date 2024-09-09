# ArduPilot Backend research resources for Pegasus Simulator Extension 🚀

This document serves as a comprehensive repository of resources, links, and commands utilized in the implementation of Ardupilot backend support for the Pegasus Simulator extension in Isaac Sim. It encompasses various aspects of the research process, including environment setup, and relevant documentation references.

## Pegasus Environment Setup 🛠️

```bash
# Link the application
./link_app.sh --path $ISAACSIM_PATH

# Launch Isaac Sim with the Pegasus simulator enabled
./app/isaac-sim.sh --ext-folder extensions --enable pegasus.simulator

# Set the ISAACSIM_PATH environment variable
export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac-sim-4.0.0"

# Setup Python environment
source ${ISAACSIM_PATH}/setup_python_env.sh

# Define aliases
alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
alias ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"
```

## Research & Setup Resources 📚

### PX4 SITL 🚁

PX4 Simulation supports sensor injection over MAVLink, while ArduPilot does not.
- [PX4 SITL Documentation](https://docs.px4.io/main/en/simulation/#sitl-simulation-environment)

## QGroundControl 🎛️
- [Pegasus Simulator Tutorial](https://pegasussimulator.github.io/PegasusSimulator/source/tutorials/run_extension_mode.html)
- [QGroundControl Installation Guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

### Ardupilot SITL 🛩️
- [ArduPilot Building Setup](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
- [ArduPilot SITL Setup](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html#setting-up-sitl-on-linux)
- [Using SITL for ArduPilot Testing](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#using-sitl-for-ardupilot-testing)

```bash
# Install SITL dependencies
sudo apt install python3-matplotlib python3-serial python3-wxgtk4.0 wxpython-tools python3-lxml python3-scipy python3-opencv ccache gawk python3-pip python3-pexpect
```

```bash
# Clone the ArduPilot repository
git clone https://github.com/Ardupilot/ardupilot ~

# Navigate to the ArduPilot directory
cd ~/ardupilot

# Initialize submodules
git submodule update --init

# Install prerequisites
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Reload profile and clean previous builds
. ~/.profile
./waf clean

# Run ArduCopter SITL
cd ArduCopter
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map

# Run ArduCopter SITL without rebuilding
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild
```

### Ardupilot_Gazebo 🏗️

- [Migrating ArduPilot Plugin for Gazebo](https://github.com/gazebosim/gz-sim/blob/gz-sim8/tutorials/migrating_ardupilot_plugin.md?plain=1)

For each UAV model in simulation, there is one instance of `ArduPilotPlugin` loaded into the simulation process.

> **Note:** The plugin uses internal simulation APIs to retrieve the UAV's current state, which it sends to an external ArduPilot process via a custom UDP protocol (Flight Dynamics Model, or FDM). The ArduPilot process makes the vehicle state available via MAVLink to other processes, such as QGroundControl (QGC). Commands in QGC like "take off" or "goto waypoint" are sent via MAVLink to ArduPilot, which computes motor commands and sends them to the plugin, which passes them onto the vehicle via internal simulation APIs.

## Ardupilot Gazebo Patch for Ubuntu 22.04
In `CMakeLists.txt`, change `std-c++14` to `-std=c++17`.

## Gazebo Garden 🌱
- [Gazebo Garden Installation](https://gazebosim.org/docs/latest/install_ubuntu/)
- [Gazebo API Documentation](https://gazebosim.org/api/sim/8/install.html)

## Gazebo11 Classic 🏛️
- [Gazebo11 Classic Download](https://classic.gazebosim.org/download)
- [ArduPilot SITL with Gazebo Legacy](https://ardupilot.org/dev/docs/sitl-with-gazebo-legacy.html#sitl-with-gazebo-legacy)

```bash
# Update and install Gazebo
sudo apt-get update
sudo apt-get install lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

Updated dependencies for Ubuntu 22.04:
- [ArduPilot Gazebo GitHub Repository](https://github.com/JerichoGroup/ardupilot_gazebo/tree/gazebo11-ubuntu22)

### Ardupilot Research 🧪
> **Note:** Connection to SITL is made via a UDP link. The physics backend should listen for incoming messages on port 9002. The simulation should reply to the IP and port the messages were received from. This is handled by `libAP_JSON`. This removes the need to configure the target IP and port for SITL in the physics backend. ArduPilot SITL will send an output message every 10 seconds allowing the physics backend to auto-detect.

- [Copter-4.5.5 JSON Examples](https://github.com/ArduPilot/ardupilot/tree/0dbe9ed27fb28bfeee4063b5cf2634b851e6a690/libraries/SITL/examples/JSON)


### MAVLink Messages 📡
- [MAVLink Messages](https://mavlink.io/en/messages/common.html)
- `HIL_ACTUATOR_CONTROLS`
- `SERVO_OUTPUT_RAW` - units (us) (microseconds)

### ArduPilot Frames 📏
- [Connecting ESCs and Motors](https://ardupilot.org/copter/docs/connect-escs-and-motors.html)
- [Frame Type Parameters](https://ardupilot.org/copter/docs/parameters.html#frame-type)
- [ArduPilot Vehicle Info](https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/pysim/vehicleinfo.py)

### AirSim SITL Helper 🦈
- [SITL with AirSim](https://ardupilot.org/dev/docs/sitl-with-airsim.html)
- [ArduPilot SITL Diagram](https://ardupilot.org/dev/_images/ArdupilotSoftwareintheLoopSITL.jpg)
```bash
sim_vehicle.py -v ArduCopter -f airsim-copter --console --map -A "--sim-port-in=9003 --sim-port-out=9002"
```

### WireShark MAVLink Filter 🦈
- [Wireshark MAVLink Guide](https://mavlink.io/en/guide/wireshark.html)
- [MAVLink Installation](https://mavlink.io/en/getting_started/installation.html)
- [Generate MAVLink Libraries](https://mavlink.io/en/getting_started/generate_libraries.html)

```bash
# Clone MAVLink repository and install dependencies
git clone https://github.com/mavlink/mavlink.git --recursive
cd mavlink
python3 -m pip install -r pymavlink/requirements.txt

# Generate MAVLink Lua scripts
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=mavlink_2_common message_definitions/v1.0/common.xml
# Replace to your Wireshark plugin directory
sudo cp mavlink_2_common.lua /usr/lib/x86_64-linux-gnu/wireshark/plugins/

# OR

python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=mavlink_2_ardupilotmega message_definitions/v1.0/ardupilotmega.xml
sudo cp mavlink_2_ardupilotmega.lua /usr/lib/x86_64-linux-gnu/wireshark/plugins/
```

#### Wireshark Filters 🔍
```bash
mavlink_proto 
mavlink_proto && (tcp.port == 5760 || tcp.port == 14500 || udp.port == 5760 || udp.port == 14500)
```
## IsaacSim Notes ⚙️
It is important to note that MAVLink logic may block IsaacSim's UI and affect FPS. Avoid using `blocking=True` and `time.sleep` is not recommended.
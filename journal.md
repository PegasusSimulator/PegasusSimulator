# Resources:
https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux
https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html#setting-up-sitl-on-linux
https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#using-sitl-for-ardupilot-testing

# Ardupilot SITL
```
git clone https://github.com/Ardupilot/ardupilot ~
cd ~/ardupilot
git submodule update --init
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf clean

cd ArduCopter
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild
```

# PX4 SITL
PX4 Simulation support sensor injection over mavlink, while ardupilot doesn't!
https://docs.px4.io/main/en/simulation/#sitl-simulation-environment

# QGroundControl
https://pegasussimulator.github.io/PegasusSimulator/source/tutorials/run_extension_mode.html
https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html


# Ardupilot_Gazebo

https://github.com/gazebosim/gz-sim/blob/gz-sim8/tutorials/migrating_ardupilot_plugin.md?plain=1
```
For each UAV model in simulation, there is one instance of ArduPilotPlugin
loaded into the simulation process.

!!!
    That plugin uses internal simulation APIs
    to retrieve the UAV's current state, which it sends to an external ArduPilot
    process via a custom UDP protocol (it's called Flight Dynamics Model, or FDM).
!!!

The ArduPilot process in turn makes the vehicle state available via the MAVLink
protocol to other processes, such as QGroundControl (QGC). The user can issue
commands in QGC like "take off" or "goto waypoint", which are sent via MAVLink
to ArduPilot, which computes motor commands and sends them to the plugin, which
passes them onto the vehicle via internal simulation APIs.

```


## ardupilot_gazebo patch for ubuntu 22.04
In CMakeLists.txt change std-c++14 to -std=c++17

## Gazebo Garden
https://gazebosim.org/docs/latest/install_ubuntu/
https://gazebosim.org/api/sim/8/install.html

## Gazebo11 classic
https://classic.gazebosim.org/download
https://ardupilot.org/dev/docs/sitl-with-gazebo-legacy.html#sitl-with-gazebo-legacy
Updated dependencies for ubuntu 22.04

https://github.com/JerichoGroup/ardupilot_gazebo/tree/gazebo11-ubuntu22

# Ardupilot Research:
```
    Connection to SITL is made via a UDP link. The physics backend should listen for incoming messages on port 9002. The sim should then reply to the IP and port the messages were received from. This is handled by libAP_JSON. This removes the need to configure the target IP and port for SITL in the physics backend. ArduPilot SITL will send an output message every 10 seconds allowing the physics backend to auto-detect.
```
Copter-4.5.5
https://github.com/ArduPilot/ardupilot/tree/0dbe9ed27fb28bfeee4063b5cf2634b851e6a690/libraries/SITL/examples/JSON

# airsim-sitl helper 
https://ardupilot.org/dev/docs/sitl-with-airsim.html
sim_vehicle.py -v ArduCopter -f airsim-copter --console --map -A "--sim-port-in=9003 --sim-port-out=9002"

https://ardupilot.org/dev/_images/ArdupilotSoftwareintheLoopSITL.jpg

### SITL deps:
(python3-wxgtk4.0 wxpython-tools)
sudo apt install python3-matplotlib python3-serial python3-wxgtk4.0 wxpython-tools python3-lxml python3-scipy python3-opencv ccache gawk python3-pip python3-pexpect




```
sudo apt-get update
sudo apt-get install lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

# WireShark MAVlink filter:
https://mavlink.io/en/guide/wireshark.html
https://mavlink.io/en/getting_started/installation.html
https://mavlink.io/en/getting_started/generate_libraries.html

```
git clone https://github.com/mavlink/mavlink.git --recursive
cd mavlink
python3 -m pip install -r pymavlink/requirements.txt
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=mavlink_2_common message_definitions/v1.0/common.xml
# Replace to your wireshark plugin directory
sudo cp mavlink_2_common.lua /usr/lib/x86_64-linux-gnu/wireshark/plugins/
# OR
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=mavlink_2_ardupilotmega message_definitions/v1.0/ardupilotmega.xml
sudo cp mavlink_2_ardupilotmega.lua /usr/lib/x86_64-linux-gnu/wireshark/plugins/
```
### Wireshark filters
```
mavlink_proto 
mavlink_proto && (tcp.port == 5760 || tcp.port == 14500 || udp. port == 5760 || udp.port == 14500)
```

# Mavlink messages:
https://mavlink.io/en/messages/common.html
HIL_ACTUATOR_CONTROLS
SERVO_OUTPUT_RAW - units (us) (microseconds)

# Ardupilot frames
https://ardupilot.org/copter/docs/connect-escs-and-motors.html
https://ardupilot.org/copter/docs/parameters.html#frame-type
https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/pysim/vehicleinfo.py

# General notes
It is important to notice that the logic (mavlink) is blocking the UI and affects FPS.
Do not use `blocking=True` anywhere and `time.sleep` is not recommanded.


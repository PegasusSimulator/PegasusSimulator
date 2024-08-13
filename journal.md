# Resources:
https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux
https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html#setting-up-sitl-on-linux
https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#using-sitl-for-ardupilot-testing

# Ardupilot SITL
git clone --recurse-submodules https://github.com/Ardupilot/ardupilot ~
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf clean

cd ArduCopter
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild

# PX4 SITL

# QGroundControl
https://pegasussimulator.github.io/PegasusSimulator/source/tutorials/run_extension_mode.html
https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
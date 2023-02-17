Known Issues
============

ROS 2 control backend not-working properly
------------------------------------------

At the moment, the Isaac Sim support for ROS 2 Humble (Ubuntu 22.04LTS) is still a little bit shaky. Since this extension
was developed on Ubuntu 22.04LTS, I was not able to fully test this functionality yet. Therefore, ROS 2 control backend was 
temporarily disabled in the GUI and the only control backend available when using Pegasus Simulator in extension mode is the PX4/MAVLink
one. This functionality will be re-activated as soon as it becomes stable. For the meantime, you can use the `ROS 2 interface
provided by the PX4 team <https://docs.px4.io/main/en/ros/ros2_comm.html>`__.
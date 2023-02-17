# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).


## [1.0.0] - 2023-02-17
- Initial version of Pegasus Simulator extension

### Added
- A widget GUI to spawn a limited set of simulation environments and drones using the PX4-bakend.
- A powerful sensors, drag, thrusters, control and vehicle API.
- Barometer, IMU, magnetometer and GPS sensors.
- Linear drag model.
- Quadratic thrust curve model.
- Multirotor model.
- The 3DR Iris quadrotor simulation model.
- MAVLink communications control support.
- ROS 2 communications control support (needs fixing).
- A library for implementing rotations from NED to ENU and FLU to FRD frame conventions.
- Examples on how to use the framework in standalone scripting mode.
- Demo with a nonlinear controller implemented in python.
- A PX4 tool for automatically launching PX4 in SITL mode when provided with the PX4-Autopilot installation directory.
- A paper describing the motivation for this framework and its inner-workings.
- Basic documentation generation using sphinx.
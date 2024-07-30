Roadmap
=======

In this section a basic feature roadmap is presented. The features in this roadmap are subject to
changes. There are no specific delivery dates nor priority table. Some of the unchecked features
might already be implemented but not yet documented.

If there is some feature missing that you would like to see added, please check the :ref:`Contributing` section.

* Supported sensors

  * |check_| Baromter
  * |check_| GPS
  * |check_| IMU (Accelerometer + Gyroscope)
  * |check_| Magnetometer
  * |check_| Camera (Implemented as a Graph Node)
  * |uncheck_| UDP Camera

* Supported actuators

  * |check_| Quadratic Thrust Curve

* Base vehicles

  * |check_| 3DR Iris
  * |uncheck_| Typhoon
  * |uncheck_| Fixed-wing plane
  * |uncheck_| VTOL

* API backends
 
  * |check_| Mavlink (with direct PX4 integration)
  * |check_| Direct ROS 2 interface
  * |uncheck_| Python backend for Reinforcement Learning (RL)

* UI
  
  * |check_| Select from NVIDIA samples worlds
  * |check_| Select from Pegasus sample vehicles
  * |uncheck_| Add an option to clone and compile PX4-Autopilot directly from the Pegasus Simulator UI

.. |check| raw:: html

    <input checked=""  type="checkbox">

.. |check_| raw:: html

    <input checked=""  disabled="" type="checkbox">

.. |uncheck| raw:: html

    <input type="checkbox">

.. |uncheck_| raw:: html

    <input disabled="" type="checkbox">

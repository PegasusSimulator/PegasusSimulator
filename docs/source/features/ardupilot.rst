.. _ardupilot:

ArduPilot
=====================

.. image:: https://img.shields.io/badge/IsaacSim-4.1.0-brightgreen.svg
   :alt: IsaacSim 4.1.0

.. image:: https://img.shields.io/badge/ArduPilot--Copter-4.4.0-brightgreen.svg
   :alt: ArduPilot-Copter 4.4

.. image:: https://img.shields.io/badge/Ubuntu-22.04LTS-brightgreen.svg
   :alt: Ubuntu 22.04

This project bridges between the ArduPilot project and IsaacSim, allowing the ArduPilot community to leverage IsaacSim's physics simulation and photorealistic rendering capabilities!

**Notice** `PegasusArduPilot <https://github.com/TomerTip/PegasusArduPilot>`_ by `TomerTip <https://github.com/TomerTip>`_ is a fork of PegasusSimulator, containing the ArduPilot integration feature, before merging into the main project.

Backend selection:

.. figure:: /_static/ardupilot/pegasus_backend_ui.gif
   :alt: Backend Selection
   :align: center

Drone spawn:

.. figure:: /_static/ardupilot/ardupilot_spawn.gif
   :alt: ArduPilot drone spawn
   :align: center

Drone takeoff:

.. figure:: /_static/ardupilot/drone_takeoff.gif
   :alt: ArduPilot drone takeoff
   :align: center

Camera demo:

.. figure:: /_static/ardupilot/ardupilot_camera.gif
   :alt: ArduPilot Camera Demo
   :align: center

Setup
-----

1. Follow PegasusSimulator installation steps: `PegasusSimulator Installation <https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html>`_
2. Follow ArduPilot SITL installation steps:
   - `ArduPilot SITL Installation <https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux>`_
   - `Setting up SITL on Linux <https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html#setting-up-sitl-on-linux>`_
   
   You may install ArduPilot at ``~/ardupilot``, or change the path in the UI inside Pegasus backend selection menu.

`PyArduPilotPlugin <https://github.com/TomerTip/PyArduPilotPlugin>`_
-------------------------

To get PegasusSimulator and ArduPilot SITL talking, I created a Python implementation of the custom protocol used between ArduPilot SITL and the simulator - called `PyArduPilotPlugin <https://github.com/TomerTip/PyArduPilotPlugin>`_.
This project allows developers to create a custom simulator and integrate it with ArduPilot SITL control using Python!

Changes to main Project
-----------------------

Since the code of PegasusSimulator is currently tightly coupled with `PX4`, I had to change the backend class hierarchy to become more generic and extendable. 

.. image:: /_static/ardupilot/pegasus_backends.png
   :alt: Pegasus Backends
   :align: center

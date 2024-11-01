Pegasus Simulator
#################

Overview
========

**Pegasus Simulator** is a framework built on top of `NVIDIA
Omniverse <https://docs.omniverse.nvidia.com/>`__ and `Isaac
Sim <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html>`__. It is designed to provide an easy yet
powerful way of simulating the dynamics of multirotors vehicles. It provides a simulation interface for `PX4 <https://px4.io/>`__
integration as well as custom python control interface. At the moment, only multirotor vehicles are supported, with support for other vehicle topologies planned for future versions.

.. raw:: html

   <p align = "center">
   <a href="https://youtu.be/_11OCFwf_GE" target="_blank"><img src="_static/pegasus_cover.png" alt="Pegasus Simulator image" align="center" height="50"/></a>
   <a href="https://youtu.be/_11OCFwf_GE" target="_blank"><img src="_static/mini demo.gif" alt="Pegasus Simulator gif" align="center" height="50"/></a>
   </p>

If you find ``Pegasus Simulator`` useful in your academic work, please cite the paper below. It is also available `here <https://doi.org/10.1109/ICUAS60882.2024.10556959>`_.

.. code-block:: bibtex

   @INPROCEEDINGS{10556959,
      author={Jacinto, Marcelo and Pinto, João and Patrikar, Jay and Keller, John and Cunha, Rita and Scherer, Sebastian and Pascoal, António},
      booktitle={2024 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
      title={Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation}, 
      year={2024},
      volume={},
      number={},
      pages={917-922},
      keywords={Simulation;Robot sensing systems;Real-time systems;Sensor systems;Sensors;Task analysis},
      doi={10.1109/ICUAS60882.2024.10556959}}

Latest Updates
==============

* **2024-11-01**: Pegasus Simulator v4.2.0 is released for Isaac 4.2.0. This version is **NOT** compatible with older versions of Isaac Sim. This version includes a new experimental interface for Ardupilot integration, provided by open-source contributor `Tomer Tiplitsky <https://github.com/TomerTip>`__.
* **2024-08-02**: Pegasus Simulator v4.1.0 is released for Isaac 4.1.0. This version is **NOT** compatible with older versions of Isaac Sim.

Guidance, Control and Navigation Project
========================================

In parallel to this project, the Pegasus (GNC) guidance, control, and navigation project serves as the foundation control code for performing real-world experiments for my Ph.D. More information can be found at this link:
`Pegasus GNC <https://pegasusresearch.github.io/pegasus/>`__.

Developer Team
~~~~~~~~~~~~~~

This simulation framework is an open-source effort, started by me, Marcelo Jacinto in January/2023. It is a tool that was created with the original purpose of serving my Ph.D. workplan for the next 4 years, which means that you can expect this repository to be mantained, hopefully at least until 2027.

- Project Founder
   - `Marcelo Jacinto <https://github.com/MarceloJacinto>`__, under the supervision of Prof. Rita Cunha and Prof. Antonio Pascoal (IST/ISR-Lisbon)
- Architecture
   - `Marcelo Jacinto <https://github.com/MarceloJacinto>`__
   - `João Pinto <https://github.com/jschpinto>`__
- Multirotor Dynamic Simulation and Control
   - `Marcelo Jacinto <https://github.com/MarceloJacinto>`__
- Example Applications
   - `Marcelo Jacinto <https://github.com/MarceloJacinto>`__
   - `João Pinto <https://github.com/jschpinto>`__
- Ardupilot Integration (Experimental)
   - `Tomer Tiplitsky <https://github.com/TomerTip>`__

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   source/setup/installation
   source/setup/developer

.. toctree::
   :maxdepth: 1
   :caption: Tutorials

   source/tutorials/run_extension_mode
   source/tutorials/create_standalone_application
   source/tutorials/create_standalone_simulation
   source/tutorials/create_custom_backend
   source/tutorials/create_simulation_with_people

.. toctree::
   :maxdepth: 2
   :caption: Features

   source/features/environments
   source/features/vehicles
   source/features/px4_integration
   source/features/ardupilot

.. toctree::
   :maxdepth: 2
   :caption: Source API

   source/api/index

.. toctree::
   :maxdepth: 1
   :caption: References

   source/references/contributing
   source/references/known_issues
   source/references/changelog
   source/references/roadmap
   source/references/license
   source/references/bibliography

.. automodule::"pegasus_isaac"
    :platform: Linux-x86_64
    :members:
    :undoc-members:
    :show-inheritance:
    :imported-members:
    :exclude-members: contextmanager

Other Simulation Frameworks
===========================

In this section, we acknowledge the nobel work of those who came before us and inspired this work:

- :cite:p:`gazebo` Gazebo simulator
- :cite:p:`rotorS` RotorS simulation plugin for gazebo
- :cite:p:`px4` PX4-SITL simulation plugin for gazebo
- :cite:p:`airsim` Microsoft Airsim project for Unreal Engine
- :cite:p:`flightmare` Flightmare simulator for Unity
- :cite:p:`jmavsim` jMAVSim java simulator

*"If I have seen further than others, it is by standing upon the shoulders of giants."*, Sir Isaac Newton

Project Sponsors
================

- Dynamics Systems and Ocean Robotics (DSOR) group of the Institute for Systems and Robotics (ISR), a research unit of the Laboratory of Robotics and Engineering Systems (LARSyS).
- Instituto Superior Técnico, Universidade de Lisboa

The work developed by Marcelo Jacinto and João Pinto was supported by Ph.D. grants funded by Fundação para as Ciências e Tecnologias (FCT).

.. raw:: html

   <p float="left" align="center">
      <img src="_static/dsor_logo.png" width="90" align="center" />
      <img src="_static/logo_isr.png" width="200" align="center"/> 
      <img src="_static/larsys_logo.png" width="200" align="center"/> 
      <img src="_static/ist_logo.png" width="200" align="center"/> 
      <img src="_static/logo_fct.png" width="200" align="center"/> 
   </p>

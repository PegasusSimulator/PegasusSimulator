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

If you find ``Pegasus Simulator`` useful in your academic work, please cite the paper below. It is also available `here <https://arxiv.org/abs/2307.05263>`_.

.. code-block:: bibtex

   @misc{jacinto2023pegasus,
      title={Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation}, 
      author={Marcelo Jacinto and João Pinto and Jay Patrikar and John Keller and Rita Cunha and Sebastian Scherer and António Pascoal},
      year={2023},
      eprint={2307.05263},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
    }

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

.. toctree::
   :maxdepth: 2
   :caption: Features

   source/features/environments
   source/features/vehicles
   source/features/px4_integration

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

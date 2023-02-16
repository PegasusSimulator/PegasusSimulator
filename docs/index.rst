Pegasus Simulator
#################

Overview
========

**Pegasus Simulator** is a framework built on top of `NVIDIA
Omniverse <https://docs.omniverse.nvidia.com/>`__ and `Isaac
Sim <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html>`__. It is designed to provide an easy, yet
powerfull way of simulating the dynamics of multirotors vehicles. It provides a simulation interface for PX4
integration as well as costume python control interface.

.. image:: /_static/pegasus_cover.png
        :width: 600px
        :align: center
        :alt: Pegasus Simulator cover photo

:Lead Developer and maintainer:
   Marcelo Jacinto
:Version: 1.0.0 
:Date: 2023/02/16

If you use ``PegasusSimulator`` in your work, please cite the `paper <https://arxiv.org/>`_:

.. code-block:: bibtex

   @misc{mittal2023orbit,
      author = {Marcelo Jacinto and Rita Cunha},
      title = {Pegasus Simulator},
      year = {2023},
      eprint = {arXiv:},
   }

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   source/setup/installation
   source/setup/developer

.. toctree::
   :maxdepth: 2
   :caption: Features

   source/features/vehicles
   source/features/drag_models
   source/features/thrust_models
   source/features/environments
   source/features/backends
   source/features/px4_integration
   source/features/ros2_integration

.. toctree::
   :maxdepth: 1
   :caption: Tutorials

   source/tutorials/create_custom_sensor
   source/tutorials/create_custom_thrust_model
   source/tutorials/create_custom_drag_model
   source/tutorials/create_custom_backend
   source/tutorials/create_custom_vehicle

.. toctree::
   :maxdepth: 2
   :caption: Source API

   source/api/index

.. toctree::
   :maxdepth: 1
   :caption: References

   source/references/contributing
   source/references/changelog
   source/references/roadmap
   source/references/license
   source/references/support
   source/references/bibliography

.. automodule::"pegasus_isaac"
    :platform: Linux-x86_64
    :members:
    :undoc-members:
    :show-inheritance:
    :imported-members:
    :exclude-members: contextmanager

Development
===========

We developed this extension using `Visual Studio Code <https://code.visualstudio.com/>`__, the 
recommended tool on NVIDIA's Omniverse official documentation. To setup Visual Studio Code with hot-reloading features, follow
the additional documentation pages:

* `Isaac Sim VSCode support <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/manual_standalone_python.html#isaac-sim-python-vscode>`__
* `Debugging with VSCode <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_advanced_python_debugging.html>`__

To enable a better developer experience when contributing or modifying this extension, and have access to features such as
autocomplete, we recommend linking the Pegasus Simulator repository to the current installation of ``ISAACSIM`` . For that, please
run the following script provided with the framework:

.. code:: bash

    ./link_app.sh --path $ISAACSIM_PATH

This script will create a symbolic link to ``ISAACSIM`` inside the repository. After this step, you can also launch the 
``ISAACSIM`` with the extension enabled (without using the GUI presented in Section :ref:`Installing the Pegasus Simulator`), by running:

.. code:: bash

    ./app/isaac-sim.sh --ext-folder extensions --enable pegasus.simulator

Code structure
--------------

This simulation framework is strucuture according to the following tree:

.. code:: bash
    
    PegasusSimulator:
    ├── .vscode
    ├── docs
    ├── examples
    ├── tools
    ├── extensions
    │   ├── pegasus.simulator
    │   │   ├── config
    │   │   │   ├── configs.yaml
    │   │   │   ├── extension.toml
    │   │   ├── data
    │   │   ├── docs
    │   │   ├── setup.py
    │   │   ├── pegasus
    │   │   │   ├── simulator
    │   │   │   │   ├── __init__.py
    │   │   │   │   ├── params.py
    │   │   │   │   ├── extension.py
    │   │   │   │   ├── assets
    │   │   │   │   │   ├── Robots
    │   │   │   │   │   ├── Worlds
    │   │   │   │   ├── logic
    │   │   │   │   │   ├── __init__.py
    │   │   │   │   │   ├── state.py
    │   │   │   │   │   ├── rotation.py
    │   │   │   │   │   ├── vehicle_manager.py
    │   │   │   │   │   ├── backends
    │   │   │   │   │   │   ├── backend.py
    │   │   │   │   │   │   ├── mavlink_backend.py
    │   │   │   │   │   │   ├── ros2_backend.py
    │   │   │   │   │   │   ├── tools
    │   │   │   │   │   │   │   ├── px4_launch_tool.py
    │   │   │   │   │   ├── dynamics
    │   │   │   │   │   │   ├── __init__.py
    │   │   │   │   │   │   ├── drag.py
    │   │   │   │   │   │   ├── linear_drag.py
    │   │   │   │   │   ├── sensors
    │   │   │   │   │   │   ├── __init__.py
    │   │   │   │   │   │   ├── sensor.py
    │   │   │   │   │   │   ├── gps.py
    │   │   │   │   │   │   ├── imu.py
    │   │   │   │   │   │   ├── magnetometer.py
    │   │   │   │   │   │   ├── barometer.py
    │   │   │   │   │   │   ├── geo_mag_utils.py
    │   │   │   │   │   ├── thrusters
    │   │   │   │   │   │   ├── __init__.py
    │   │   │   │   │   │   ├── thrust_curve.py
    │   │   │   │   │   │   ├── quadratic_thrust_curve.py
    │   │   │   │   │   ├── vehicles
    │   │   │   │   │   │   ├── __init__.py
    │   │   │   │   │   │   ├── vehicle.py
    │   │   │   │   │   │   ├── multirotor.py
    │   │   │   │   │   ├── interface
    │   │   │   │   │   │   ├── __init__.py
    │   │   │   │   │   │   ├── pegasus_interface.py
    │   │   │   │   ├── ui
    │   │   │   │   │   ├── __init__.py
    │   │   │   │   │   ├── ui_delegate.py
    │   │   │   │   │   ├── ui_window.py

The extensions directory contains the source code for the PegasusSimulator API and interactive GUI while the 
examples directory contains the a set of python scripts to launch standalone applications and pre-programed simulations.

Working in extension mode (Interactive GUI mode)
------------------------------------------------

As explained in NVIDIA's documentation, extensions are the standard way of developing on top of Isaac Sim and other Omniverse
tools. The core of our extension is developed in the ``logic`` and the ``ui`` modules.

.. code:: bash

    │   ├── pegasus.simulator
    │   │   ├── config
    │   │   ├── data
    │   │   ├── docs
    │   │   ├── setup.py
    │   │   ├── pegasus.simulator
    │   │   │   │   ├── __init__.py
    │   │   │   │   ├── params.py
    │   │   │   │   ├── extension.py
    │   │   │   │   ├── assets
    │   │   │   │   ├── logic
    │   │   │   │   ├── ui

The main API that allows the simulation
of dynamics, spawning assets on the world, etc. are defined in the ``logic`` module while the ``ui``

Working as a standalone application
-----------------------------------


Development
===========

We developed this extension using `Microsoft Visual Studio Code (VSCode) <https://code.visualstudio.com/>`__, the 
recommended tool on NVIDIA's Omniverse official documentation.

Setting up VSCode
-----------------

Additional documentation pages provided by NVIDIA:

* `Isaac Sim VSCode support <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/manual_standalone_python.html#isaac-sim-python-vscode>`__
* `Debugging with VSCode <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_advanced_python_debugging.html>`__

Code structure
--------------

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

Working in extension mode
-------------------------

Working as a standalone application
-----------------------------------

Contributing to repository
--------------------------

The Pegasus Simulator is an open-source effort, started by me, Marcelo Jacinto in January/2023. It is a tool that was 
created with the original purpose of serving my Ph.D. workplan for the next 4 years, which means that you can expect 
this repository to be mantained by me directly, hopefully until 2027. 

With that said, it is very likely that you will stumble upon bugs on the code or missing features. If you feel that there is
some critical feature missing and want to contribute to this project, suggest a new feature or just improve the documentation,
please check the :ref:`Contributing` section.

Sponsor the project
-------------------

At the moment, this project as it stands only has one indirect sponsor:
- Dynamics Systems and Ocean Robotics (DSOR) group (Portugal), under my Ph.D. grant funded by FCT.

If you want to be a part of this project, or sponsor my work with some graphics cards, jetson developer boards and other development
material to keep, please reach out to me directly at ``marcelo.jacinto@tecnico.ulisboa.pt``.
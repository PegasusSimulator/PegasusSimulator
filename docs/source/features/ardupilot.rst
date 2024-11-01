ArduPilot (Experimental)
========================

.. image:: https://img.shields.io/badge/IsaacSim-4.2.0-brightgreen.svg
   :target: https://developer.nvidia.com/isaac-sim
   :alt: IsaacSim 4.2.0

.. image:: https://img.shields.io/badge/ArduPilot--Copter-4.4.0-brightgreen.svg
   :target: https://github.com/ArduPilot/ardupilot
   :alt: ArduPilot-Copter 4.4

.. image:: https://img.shields.io/badge/Ubuntu-22.04LTS-brightgreen.svg
   :target: https://releases.ubuntu.com/22.04/
   :alt: Ubuntu 22.04

The Ardupilot integration is an experimental feature that bridges the gap between the ArduPilot project and IsaacSim, allowing the ArduPilot community to leverage IsaacSim's physics simulation and photorealistic rendering capabilities! **This feature is tailored at researchers that use Ardupilot instead of PX4.**

.. note:: 
   This feature is highly experimental and was developed by the open-source contributor `Tomer Tiplitsky <https://github.com/TomerTip>`_ in `PegasusArduPilot <https://github.com/TomerTip/PegasusArduPilot>`_, and merged into the main project.

To get PegasusSimulator and ArduPilot SITL talking, the open-source contributor `Tomer Tiplitsky <https://github.com/TomerTip>`_ created a Python implementation of the custom protocol used between ArduPilot SITL and the simulator - called `PyArduPilotPlugin <https://github.com/TomerTip/PyArduPilotPlugin>`_.
This project allows developers to create a custom simulator and integrate it with ArduPilot SITL control using Python!


Installing Ardupilot (Arducopter)
---------------------------------

1. Ensure that you have the PegasusSimulator already installed. If you have not, follow the `PegasusSimulator Installation <https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html>`_ steps, and then come back to this page.

2. Install the extra dependencies:

    .. code:: bash

        # Linux packages
        sudo apt install git make cmake python3-pip build-essential ccache g++ gawk wget valgrind screen python3-pexpect pkg-config libtool libxml2-dev libxslt1-dev xterm
       
        # Install this additional package to be able to see the window map and UI to interface with Ardupilot
        sudo apt-get install python3-wxgtk4.0 -y --no-install-recommends

        # Python packages
        pip install pymavlink MAVProxy kconfiglib jinja2 empy jsonschema pyros-genmsg packaging toml numpy future future lxml pymavlink pyserial geocoder empy==3.3.4 ptyprocess dronecan flake8 junitparser pygame intelhex --user 

3. Clone `Ardupilot <https://github.com/ArduPilot/ardupilot>`__ into your home directory:

    .. code:: bash

        # Option 1: With HTTPS
        git clone https://github.com/ArduPilot/ardupilot.git
        # Option 2: With SSH (you need to setup a github account with ssh keys)
        git clone git@github.com:ArduPilot/ardupilot.git

4. Checkout to the stable version and compile the code for software-in-the-loop (SITL) mode:

    .. code:: bash
        
        # Go to the PX4 directory
        cd ardupilot

        # Checkout to the latest stable release
        git checkout ArduCopter-stable

        # Initiate all the submodules. Note this will download modules such as SITL-gazebo which we do not need
        # but this is the safest way to make sure that the PX4-Autopilot and its submodules are all checked out in 
        # a stable and well tested release
        git submodule update --init --recursive

        # Generate the configuration for compilation
        ./waf configure --board MatekH743

        # Compile the code
        ./waf copter

        # Go to the tools folder and run the python script to setup the simulation
        cd Tools/autotest

        # Run the script to setup the simulation
        python3 sim_vehicle.py -v copter --console --map -w

   
In the end, a white terminal should pop-up. Press Ctrl+C to exit and close the window.

.. note:: 
   If you did not install ArduPilot at ``~/ardupilot``, you can also set your custom installation path inside the Pegasus Simulator GUI, or by editing the file ``PegasusSimulator/extensions/pegasus/simulator/config/config.yaml`` and setting the ``ardupilot_dir`` field to the correct path.


This installation guide was based on the following resources:
   - `ArduPilot SITL Installation <https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux>`_
   - `Setting up SITL on Linux <https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html#setting-up-sitl-on-linux>`_

Running a Simulation with Ardupilot (GUI Mode)
----------------------------------------------

1. Open ``ISAACSIM``, either by using the Omniverse Launcher or the terminal command:

   .. code:: bash

      ISAACSIM

2. Make sure the Pegasus Simulator Extension is enabled.

   .. image:: /_static/pegasus_inside_extensions_menu.png
      :width: 600px
      :align: center
      :alt: Enable the Pegasus Simulator extension inside Isaac Sim

3. Select the appropriate control backend and drone model.

   .. image:: /_static/ardupilot/pegasus_backend_ui.gif
      :alt: Backend Selection
      :align: center

3. On the new terminal that was opened, run the following commands to perform a takeoff:

   .. code:: bash

      mode guided
      arm throttle
      takeoff 3

Drone spawn:

.. image:: /_static/ardupilot/ardupilot_spawn.gif
   :alt: ArduPilot drone spawn
   :align: center

Drone takeoff:

.. image:: /_static/ardupilot/drone_takeoff.gif
   :alt: ArduPilot drone takeoff
   :align: center

Camera demo:

.. image:: /_static/ardupilot/ardupilot_camera.gif
   :alt: ArduPilot Camera Demo
   :align: center


Ardupilot Integration Architecture
----------------------------------

The Ardupilot integration is composed of two main components: the Ardupilot Mavlink Backend and the Ardupilot Mavlink Plugin. See the diagram below for a high-level overview of the architecture:

.. image:: /_static/ardupilot/pegasus_backends.png
   :alt: Pegasus Backends
   :align: center

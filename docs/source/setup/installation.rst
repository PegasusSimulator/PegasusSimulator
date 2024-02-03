Installation
============

Installing NVIDIA Isaac Sim
---------------------------

.. image:: https://img.shields.io/badge/IsaacSim-2023.1.1-brightgreen.svg
   :target: https://developer.nvidia.com/isaac-sim
   :alt: IsaacSim 2023.1.1

.. image:: https://img.shields.io/badge/PX4--Autopilot-1.14.1-brightgreen.svg
   :target: https://github.com/PX4/PX4-Autopilot
   :alt: PX4-Autopilot 1.14.1

.. image:: https://img.shields.io/badge/Ubuntu-22.04LTS-brightgreen.svg
   :target: https://releases.ubuntu.com/22.04/
   :alt: Ubuntu 22.04

.. note::
	We have tested Pegasus Simulator with Isaac Sim 2023.1.1 release on Ubuntu 22.04LTS with NVIDIA driver 545.23.08. The PX4-Autopilot used during development was v.14.1. Older versions Ubuntu and PX4-Autopilot were not tested. This extension was not tested on Windows. 

In order to install Isaac Sim on linux, download the `Omniverse AppImage here <https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage>`__ or run the following line on the terminal:

.. code:: bash

    wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage


A short video with the installation guide for Pegasus Simulator is also available `here <https://youtu.be/YCp5E8nazag>`__.

    ..  youtube:: YCp5E8nazag
        :width: 100%
        :align: center
        :privacy_mode:


Configuring the environment variables
-------------------------------------

NVIDIA provides Isaac Sim with its own Python interpreter along with some basic extensions such as numpy and pytorch. In
order for the Pegasus Simulator to work, we require the user to use the same python environment when starting a simulation
from python scripts. As such, we recommend setting up a few custom environment variables to make this process simpler.

.. note::
    Although it is possible to setup a virtual environment following the 
    instructions `here <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_python.html>`__, this
    feature was not tested.

Start by locating the `Isaac Sim installation <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_python.html>`__ 
by navigating to Isaac Sim's root folder. Typically, in Linux, this folder can be found under ``${HOME}/.local/share/ov/pkg/isaac_sim-*``,
where the ``*`` is the version number.

Add the following lines to your ``~/.bashrc`` or ``~/.zshrc`` file.

.. code:: bash

   # Isaac Sim root directory
   export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac_sim-*"
   # Isaac Sim python executable
   alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
   # Isaac Sim app
   alias ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"

If you only have one version of Isaac Sim installed, you can leave the ``*``, otherwise you will have to replace it by the 
version that you desire to use. In the remaining of the documentation, we will refer to the Isaac Sim's path as ``ISAACSIM_PATH`` ,
the provided python interpreter as ``ISAACSIM_PYTHON`` and the simulator itself as ``ISAACSIM`` .

Running Isaac Sim
~~~~~~~~~~~~~~~~~

At this point, you are expected to have NVIDIA Isaac Sim fully installed and working. To make sure you have everything setup correctly,
open a new terminal window (**Ctrl+Alt+T**), and test the following commands:

- Check that the simulator app can be launched

    .. code:: bash

        # Run the simulator with the --help argument to see all available options
        ISAACSIM --help

        # Run the simulator. A new window should open
        ISAACSIM

- Check that you can launch the simulator from a python script (standalone mode)

    .. code:: bash

        # Run the bundled python interpreter and see if it prints on the terminal "Hello World."
        ISAACSIM_PYTHON -c "print('Hello World.')"

        # Run the python interpreter and check if we can run a script that starts the simulator and adds cubes to the world
        ISAACSIM_PYTHON ${ISAACSIM_PATH}/standalone_examples/api/omni.isaac.core/add_cubes.py

If you were unable to run the commands above successfuly, then something is incorrectly configured. 
Please do not proceed with this installation until you have everything setup correctly.

Addtional Isaac Sim resources:
- `Troubleshooting documentation <https://docs.omniverse.nvidia.com/app_isaacsim/prod_kit/linux-troubleshooting.html>`__

Installing the Pegasus Simulator
--------------------------------

Clone the `Pegasus Simulator <https://www.github.com/PegasusSimulator/PegasusSimulator.git>`__:

.. code:: bash

    # Option 1: With HTTPS
    git clone https://github.com/PegasusSimulator/PegasusSimulator.git
    # Option 2: With SSH (you need to setup a github account with ssh keys)
    git clone git@github.com:PegasusSimulator/PegasusSimulator.git
    

The Pegasus Simulator was originally developed as an Isaac Sim extension with an interactive GUI, but also provides a powerfull
API that allows it to run as a standalone app, i.e. by creating python scritps (as shown in the examples directory of this repository).
To be able to use the extension in both modes, follow these steps:

1. Launch ``ISAACSIM`` application.

2. Open the Window->extensions on the top menubar inside Isaac Sim.

    .. image:: /_static/extensions_menu_bar.png
        :width: 600px
        :align: center
        :alt: Extensions on top menubar

3. On the Extensions manager menu, we can enable or disable extensions. By pressing the settings button, we can 
add a path to the Pegasus-Simulator repository.

    .. image:: /_static/extensions_widget.png
        :width: 600px
        :align: center
        :alt: Extensions widget

4. The path inserted should be the path to the repository followed by ``/extensions``.

    .. image:: /_static/ading_extension_path.png
        :width: 600px
        :align: center
        :alt: Adding extension path to the extension manager

5. After adding the path to the extension, we can enable the Pegasus Simulator extension on the third-party tab.

    .. image:: /_static/pegasus_inside_extensions_menu.png
        :width: 600px
        :align: center
        :alt: Pegasus Extension on the third-party tab

When enabling the extension for the first time, the python requirements should be install automatically for the build in 
``ISAACSIM_PYTHON`` , and after a few seconds, the Pegasus widget GUI should pop-up.

6. The Pegasus Simulator window should appear docked to the bottom-right section of the screen.

    .. image:: /_static/pegasus_gui_example.png
        :width: 600px
        :align: center
        :alt: Pegasus Extension GUI after install

Installing the extension as a library
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In order to be able to use the Pegasus Simulator API from python scripts and standalone apps, we must install this 
extension as a ``pip`` python module for the built-in ``ISAACSIM_PYTHON`` to recognize. For that, run:

.. code:: bash

        # Go to the repository of the pegasus simulator
        cd PegasusSimulator

        # Go into the extensions directory
        cd extensions

        # Run the pip command using the built-in python interpreter
        ISAACSIM_PYTHON -m pip install --editable pegasus.simulator

We use the ``--editable`` flag so that the content of the extension is linked instead of copied. After this step, you 
should be able to run the python standalone examples inside the ``examples`` folder.

Installing PX4-Autopilot
------------------------

In this first version of the Pegasus Simulator (in extension mode), the GUI widget provided is only usefull if you intend to use the PX4-Autopilot.
To install PX4-Autopilot, follow the following steps:

1. Install the dependencies (to be able to compile PX4-Autopilot):

    .. code:: bash

        # Linux packages
        sudo apt install git make cmake python3-pip
       
        # Python packages
        pip install kconfiglib jinja2 empy jsonschema pyros-genmsg packaging toml numpy future

2. Clone the `PX4-Autopilot <https://github.com/PX4/PX4-Autopilot>`__:

    .. code:: bash

        # Option 1: With HTTPS
        git clone https://github.com/PX4/PX4-Autopilot.git
        # Option 2: With SSH (you need to setup a github account with ssh keys)
        git clone git@github.com:PX4/PX4-Autopilot.git

3. Checkout to the stable version 1.14.1 and compile the code for software-in-the-loop (SITL) mode:

    .. code:: bash
        
        # Go to the PX4 directory
        cd PX4-Autopilot

        # Checkout to the latest stable release
        git checkout v1.14.1

        # Initiate all the submodules. Note this will download modules such as SITL-gazebo which we do not need
        # but this is the safest way to make sure that the PX4-Autopilot and its submodules are all checked out in 
        # a stable and well tested release
        git submodule update --init --recursive

        # Compile the code in SITL mode
        make px4_sitl_default none

Note: If you are installing a version of PX4 prior to v1.14.1, you may need to go to change the default airframe to 
be used by PX4. This can be achieved by:

    .. code:: bash
        
        # Go inside the config folder of the pegasus simulator extension
        cd PegasusSimulator/extensions/pegasus/simulator/config

        # Open the file configs.yaml
        nano configs.yaml

        # And change the line:
        px4_default_airframe: iris

You can also set the PX4 installation path inside the Pegasus Simulator GUI, as shown in the next section, or by editing
the file ``PegasusSimulator/extensions/pegasus/simulator/config/config.yaml`` and setting the ``px4_dir`` field to the correct path.

Setting the PX4 path inside the Pegasus Simulator
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The simulator provides a feature to auto-launch PX4-Autopilot for every vehicle that is spawned in the simulation world. 
For this feature to work, we need to tell the Pegasus Simulator extension where the PX4-Autopilot directory can be found. 
For that, edit the ``PX4 Path`` text field if is not correct by default and press the ``Make Default`` button. This 
field supports relative paths to the home directory, which means that you can use ``~`` to represent the home directory 
without hard-coding it.

.. image:: /_static/pegasus_GUI_px4_dir.png
    :width: 600px
    :align: center
    :alt: Pegasus GUI with px4 directory highlighted

By default, the extension assumes that PX4-Autopilot is installed at ``~/PX4-Autopilot`` .

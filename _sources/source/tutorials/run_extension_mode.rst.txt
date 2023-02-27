Run in Extension Mode (GUI)
=======================================

This tutorial introduces how to interface with the Pegasus Simulator working in extension mode, i.e. with an interactive GUI. 
This means that ``ISAACSIM`` will be launched as a standard application and the Pegasus Simulator extension should be 
enabled from the extension manager.

0. Preparation
--------------

Before you proceed, check the :ref:`Installing the Pegasus Simulator` section first, if you haven't already. You will
also require `QGroundControl <http://qgroundcontrol.com/>`__ to control the vehicle. You haven't download it yet, you can
do it `here <https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html>`__ or follow these steps:

1. Open a new terminal and run:

    .. code:: bash

        sudo usermod -a -G dialout $USER
        sudo apt-get remove modemmanager -y
        sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
        sudo apt install libqt5gui5 -y
        sudo apt install libfuse2 -y

2. Download the `QGroundControl App Image <https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage>`__.

3. Make the App image executable

    .. code:: bash

        chmod +x ./QGroundControl.AppImage

4. Execute QGroundControl by double-clicking or running:

    .. code:: bash

        ./QGroundControl.AppImage

1. Simulation Steps
-------------------

1. Open ``ISAACSIM``, either by using the Omniverse Launcher or the terminal command:

    .. code:: bash

        ISAACSIM

2. Make sure the Pegasus Simulator Extension is enabled.

    .. image:: /_static/pegasus_inside_extensions_menu.png
        :width: 600px
        :align: center
        :alt: Enable the Pegasus Simulator extension inside Isaac Sim

3. On the Pegasus Simulator tab in the bottom-right corner, click on the ``Load Scene`` button.

    .. image:: /_static/tutorials/load_scene.png
        :width: 600px
        :align: center
        :alt: Load a 3D Scene

4. Again, on the Pegasus Simulator tab, click on the ``Load Vehicle`` button.

    .. image:: /_static/tutorials/load_vehicle.png
        :width: 600px
        :align: center
        :alt: Load the vehicle

5. Press the ``play`` button on the simulator's control bar on the left corner.

    .. image:: /_static/tutorials/play.png
        :width: 600px
        :align: center
        :alt: Start the simulation environment

6. On QGroundControl, an arrow representing the vehicle should pop-up. You can now perform a take-off, but pressing the
``take-off`` button on top-left corner of QGroundControl.

    .. image:: /_static/tutorials/take_off.png
        :width: 600px
        :align: center
        :alt: Perform a take-off with the drone

7. On QGroundControl, left-click on a place on the map, press ``Go to location`` and slide at the bottom of the screen
to confirm the target waypoint for the drone to follow.

    .. image:: /_static/tutorials/go_to_location.png
        :width: 600px
        :align: center
        :alt: Perform a go-to waypoint with the drone

Congratulations 🎉️🎉️🎉️ ! You have just completed your first tutorial and you should now see the vehicle moving on the screen.

A short video of this tutorial is also available `here <https://youtu.be/_11OCFwf_GE>`__.

    ..  youtube:: _11OCFwf_GE
        :width: 100%
        :align: center
        :privacy_mode:

.. note::

    Everything that you can do using the provided GUI can also be achieved by the Pegasus Simulator API in Python. In the next
    tutorials we will cover how to create standalone Python scripts to perform simulations.
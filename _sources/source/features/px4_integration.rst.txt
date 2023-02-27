PX4 Integration
===============

The ``PX4-Autopilot`` support is provided by making use of the ``Control Backends API`` , and implementing a custom 
``MavlinkBackend`` which contains a built-in tool to launch and kill PX4 in SITL mode automatically.

To instantiate a ``MavlinkBackend`` via Python scripting, consider the following example:

.. code:: Python

    # Import the Mavlink backend module
    from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig

    # Create the multirotor configuration
    # In this example we are showing the default parameters that are used if you do not specify them
    mavlink_config = MavlinkBackendConfig({"vehicle_id": 0,
        "connection_type": "tcpin",
        "connection_ip": "localhost",
        # The actual port that gets used = "connection_baseport" + "vehicle_id"
        "connection_baseport": 4560,
        "enable_lockstep": True,
        "num_rotors": 4,
        "input_offset": [0.0, 0.0, 0.0, 0.0],
        "input_scaling": [1000.0, 1000.0, 1000.0, 1000.0],
        "zero_position_armed": [100.0, 100.0, 100.0, 100.0],
        "update_rate": 250.0,

        # Settings for automatically launching PX4
        # If px4_autolaunch==False, then "px4_dir" and "px4_vehicle_model" are unused
        "px4_autolaunch": True,
        "px4_dir": "PegasusInterface().px4_path",
        "px4_vehicle_model": "iris",
        })
    config_multirotor.backends = [MavlinkBackend(mavlink_config)]

.. note::

    In general, the Pegasus Simulator does not need to know where you have PX4 running to simulate the vehicle and send data 
    through ``MAVLink`` . However, if you intend to use the provided ``PX4 auto-launch`` feature, you must inform Pegasus Simulator
    where you have your local install of PX4.

By default, the simulator expects PX4 to be located at ``~/PX4-Autopilot`` directory. You can set the default 
path for the ``PX4-Autopilot`` by either:

1. Using the GUI of the Pegasus Simulator when operating in extension mode.


    .. image:: /_static/pegasus_GUI_px4_dir.png
        :width: 600px
        :align: center
        :alt: Setting the PX4 path

2. Use the methods provided by :class:`PegasusInterface`, i.e:

    .. code:: Python

        from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

        # Start the Pegasus Interface
        pg = PegasusInterface()

        # Set the default PX4 installation path used by the simulator
        # This will be saved for future runs
        pg.set_px4_path("path_to_px4_directory")


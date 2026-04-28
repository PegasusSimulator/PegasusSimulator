#!/usr/bin/env python
"""
| File: 9_people.py
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API to run a simulation
| where people move around in the world.
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World
from isaacsim.core.utils.extensions import enable_extension

# Enable/disable ROS bridge extensions to keep only ROS2 Bridge
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.code_editor.vscode")

# Update the simulation app with the new extensions
simulation_app.update()

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

# -------------------------------------------------------------------------------------------------
# Define the PegasusApp class where the simulation will be run
# -------------------------------------------------------------------------------------------------
class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics,
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        #self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Full Warehouse"])

        config_multirotor = MultirotorConfig()
        # Create the multirotor configuration
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
        })

        config_multirotor.backends = [
            PX4MavlinkBackend(mavlink_config),
            ROS2Backend(vehicle_id=1, 
                config={
                    "namespace": 'drone', 
                    "pub_sensors": True,
                    "pub_graphical_sensors": True,
                    "pub_state": True,
                    "pub_tf": True,
                    "sub_control": False,
                    "use_sim_time": False
                }
            )
        ]

        # Add camera and lidar sensors
        # MID360 Lidar sensor attributes
        # see: https://forums.developer.nvidia.com/t/livox-mid360/283074/7
        mid360_attributes = {
            'omni:sensor:Core:scanType': "ROTARY",
            'omni:sensor:Core:intensityProcessing': "NORMALIZATION",
            'omni:sensor:Core:rotationDirection': "CW",
            'omni:sensor:Core:rayType': "IDEALIZED",
            'omni:sensor:Core:nearRangeM': 0.1,
            'omni:sensor:Core:farRangeM':40.0,
            'omni:sensor:Core:rangeResolutionM': 0.004,
            'omni:sensor:Core:rangeAccuracyM': 0.025,
            'omni:sensor:Core:avgPowerW': 0.002,
            'omni:sensor:Core:minReflectance': 0.1,
            'omni:sensor:Core:minReflectanceRange': 70.0,
            'omni:sensor:Core:wavelengthNm': 905.0,
            'omni:sensor:Core:pulseTimeNs': 6,
            'omni:sensor:Core:azimuthErrorMean': 0.1,
            'omni:sensor:Core:azimuthErrorStd': 0.5,
            'omni:sensor:Core:elevationErrorMean': 0.1,
            'omni:sensor:Core:elevationErrorStd': 0.5,
            'omni:sensor:Core:maxReturns': 2,
            'omni:sensor:Core:scanRateBaseHz': 20.0,
            'omni:sensor:Core:reportRateBaseHz': 7761,
            'omni:sensor:Core:numberOfEmitters': 40,
            'omni:sensor:Core:numberOfChannels': 40,
            'omni:sensor:Core:rangeOffset': 0.03,
            'omni:sensor:Core:intensityMappingType': "LINEAR",
            'omni:sensor:Core:emitterState:s001:azimuthDeg': [0]*40,
            'omni:sensor:Core:emitterState:s001:elevationDeg': [
                -7.0, -5.525, -4.050, -2.575, -1.1004, 0.374, 1.849, 3.324, 4.799, 6.274,
                7.7494, 9.2249, 10.699, 12.174, 13.645, 15.1243, 16.5999, 18.074, 19.5499, 21.024,
                22.493, 23.9749, 25.44, 26.924, 28.39, 29.8743, 31.3499, 32.824, 34.29, 35.774,
                37.2486, 38.724, 40.19, 41.674, 43.14, 44.624, 46.09, 47.574, 49.048, 50.524
            ],
            'omni:sensor:Core:emitterState:s001:fireTimeNs': [i*1000 for i in range(40)],
            'omni:sensor:Core:emitterState:s001:distanceCorrectionM': [0.0]*40,
            'omni:sensor:Core:emitterState:s001:focalDistM': [0.0]*40,
            'omni:sensor:Core:emitterState:s001:focalSlope': [0.0]*40,
            'omni:sensor:Core:emitterState:s001:horOffsetM': [0.0]*40,
            'omni:sensor:Core:emitterState:s001:reportRateDiv': [0.0]*40,
            'omni:sensor:Core:emitterState:s001:vertOffsetM': [0.0]*40,
            'omni:sensor:Core:emitterState:s001:channelId': list(range(1, 41)),
        }

        config_multirotor.graphical_sensors = [
            MonocularCamera("camera", config={
                "position": [0.30, 0.0, 0.0],
                "orientation": [0.0, 0.0, 180.0]
            }),
            Lidar("lidar", config={
                "position": [0.0, 0.0, 0.045],
                "orientation": [0.0, 0.0, 0.0],
                #"sensor_configuration": "Example_Rotary_2D",
                #"sensor_configuration": "Example_Rotary",
                #"sensor_configuration": "HESAI_XT32_SD10",
                "sensor_configuration": "OS1_REV6_32ch20hz1024res",
                "sensor_attributes": mid360_attributes
            })
        ]

        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 90.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()

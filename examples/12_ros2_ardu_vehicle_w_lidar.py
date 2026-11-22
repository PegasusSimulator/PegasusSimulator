#!/usr/bin/env python
"""
| File: 12_ros2_ardu_vehicle_w_lidar.py
| Author: Bethan Edwards
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Simulates an Iris drone with Ros2 and ArduPilot backends, + Camera and Lidar.
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

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import (ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig)
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

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
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        self.vehicle_factory(1)

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def vehicle_factory(self, vehicle_id: int):
        """Auxiliar method to create multiple multirotor vehicles

        Args:
            vehicle_id (_type_): _description_
        """
        # Create the backends (ArduPilot and ROS2)
        config_multirotor = MultirotorConfig()

        backend_config = ArduPilotMavlinkBackendConfig({
            "vehicle_id": vehicle_id,
            "ardupilot_autolaunch": True,
            "ardupilot_dir": self.pg.ardupilot_path,
            "ardupilot_vehicle_model": "gazebo-iris"
        })
        config_multirotor.backends = [
            ArduPilotMavlinkBackend(config=backend_config),
            ROS2Backend(vehicle_id=vehicle_id,
                        config={
                            "namespace": f'drone',  # vehicle_id is added here by default
                            "pub_sensors": True,
                            "pub_graphical_sensors": True,
                            "pub_state": True,
                            "sub_control": False,
                            "pub_tf": True,
                        })
        ]

        # Add camera and lidar sensor
        # MID360 Lidar sensor attributes
        # see: https://forums.developer.nvidia.com/t/livox-mid360/283074/7
        mid360_attributes = {
            'omni:sensor:Core:scanType': "ROTARY",
            'omni:sensor:Core:intensityProcessing': "NORMALIZATION",
            'omni:sensor:Core:rotationDirection': "CW",
            'omni:sensor:Core:rayType': "IDEALIZED",
            'omni:sensor:Core:nearRangeM': 0.1,
            'omni:sensor:Core:farRangeM': 40.0,
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
            'omni:sensor:Core:emitterState:s001:azimuthDeg': [0] * 40,
            'omni:sensor:Core:emitterState:s001:elevationDeg': [
                -7.0, -5.525, -4.050, -2.575, -1.1004, 0.374, 1.849, 3.324, 4.799, 6.274,
                7.7494, 9.2249, 10.699, 12.174, 13.645, 15.1243, 16.5999, 18.074, 19.5499, 21.024,
                22.493, 23.9749, 25.44, 26.924, 28.39, 29.8743, 31.3499, 32.824, 34.29, 35.774,
                37.2486, 38.724, 40.19, 41.674, 43.14, 44.624, 46.09, 47.574, 49.048, 50.524
            ],
            'omni:sensor:Core:emitterState:s001:fireTimeNs': [i * 1000 for i in range(40)],
            'omni:sensor:Core:emitterState:s001:distanceCorrectionM': [0.0] * 40,
            'omni:sensor:Core:emitterState:s001:focalDistM': [0.0] * 40,
            'omni:sensor:Core:emitterState:s001:focalSlope': [0.0] * 40,
            'omni:sensor:Core:emitterState:s001:horOffsetM': [0.0] * 40,
            'omni:sensor:Core:emitterState:s001:reportRateDiv': [0.0] * 40,
            'omni:sensor:Core:emitterState:s001:vertOffsetM': [0.0] * 40,
            'omni:sensor:Core:emitterState:s001:channelId': list(range(1, 41)),
        }

        #3d lidar running in simulation time
        lidar_3d = Lidar("lidar_3d",
                  config={
                      "position": [0.0, 0.0, -0.5],
                      "orientation": [0.0, 0.0, 0.0],
                      "sensor_configuration": {
                          "sensor_configuration": "Example_Rotary",
                      },
                      "sensor_attributes": mid360_attributes,
                      "show_render": False,
                      "use_sys_time": False 
                      #Option to use system time instead of simulation. Defaults to sim if omitted.
                  })

        #2d lidar running in system time
        lidar_2d = Lidar("lidar_2d",
                    config={
                        "position": [0.0, 0.0, 0.5],
                        "orientation": [0.0, 0.0, 0.0],
                        "sensor_configuration": {
                            "sensor_configuration": "Example_Rotary_2D",
                        },
                        "sensor_attributes": mid360_attributes,
                        "use_sys_time": True
                        # Option to use system time instead of simulation. Defaults to sim if omitted.
                    })

        camera = MonocularCamera("camera", config={
                    "position": [0.30, 0.0, 0.0],
                    "orientation": [0.0, 0.0, 180.0]
                })

        config_multirotor.graphical_sensors = [
            lidar_3d,
            lidar_2d,
            camera
        ]
            
        #now Initialize the Iris
        vehicle = Multirotor(
            f"/World/drone{vehicle_id}",
            ROBOTS['Iris'],
            vehicle_id,
            [0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor
        )

        #have to reinitialize lidar sensors here, or they won't work.
        lidar_3d.initialize(vehicle)
        lidar_2d.initialize(vehicle)

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

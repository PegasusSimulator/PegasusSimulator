#!/usr/bin/env python
"""
| File: 8_camera_vehicle.py
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto and Filip Stec. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API to run a simulation
with a single vehicle equipped with a camera, producing rgb and camera info ROS2 topics.
"""

# Imports to start Isaac Sim from this script
import carb
from omni.isaac.kit import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.utils.extensions import disable_extension, enable_extension

# Enable/disable ROS bridge extensions to keep only ROS2 Bridge
disable_extension("omni.isaac.ros_bridge")
enable_extension("omni.isaac.ros2_bridge")

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphs import ROS2Camera

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

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

        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor = MultirotorConfig()
        # Create the multirotor configuration
        mavlink_config = MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": "/home/marcelo/PX4-Autopilot",
            "px4_vehicle_model": 'iris'
        })
        config_multirotor.backends = [MavlinkBackend(mavlink_config)]

        # Create camera graph for the existing Camera prim on the Iris model, which can be found 
        # at the prim path `/World/quadrotor/body/Camera`. The camera prim path is the local path from the vehicle's prim path
        # to the camera prim, to which this graph will be connected. All ROS2 topics published by this graph will have 
        # namespace `quadrotor` and frame_id `Camera` followed by the selected camera types (`rgb`, `camera_info`).
        config_multirotor.graphs = [ROS2Camera("body/Camera", config={"types": ['rgb', 'camera_info']})]

        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
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

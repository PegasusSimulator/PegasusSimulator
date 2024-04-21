#!/usr/bin/env python
"""
| File: 10_single_vehicle_ros1_graphs.py
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto, Filip Stec. and Siky L. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API to run a simulation
with a single vehicle equipped with a camera, two fixed cameras and publish camera images, odometry, clock and transformation tree in ROS1 topics 
with the help of the ROS1Camera, ROS1Odom, ROS1Clock and ROS1TF graphs.
"""

# Imports to start Isaac Sim from this script
import carb
from omni.isaac.kit import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": True})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.extensions import disable_extension, enable_extension
from omni.isaac.sensor import Camera

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphs import ROS1Camera, ROS1Odom, ROS1Clock, ROS1TF

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
        mavlink_config = MavlinkBackendConfig(
            {
                "vehicle_id": 0,
                "px4_autolaunch": True,
                "px4_dir": "/shared/codes/PX4-Autopilot",
                "px4_vehicle_model": "gazebo-classic_iris",
            }
        )
        config_multirotor.backends = [MavlinkBackend(mavlink_config)]

        # Create fixed Camera
        camera_params_common = {
            "resolution": [640, 480],
            "frequency": None,
            "dt": None,
            "position": None,  # with respect to the world frame
            "translation": None,  # with respect to its parent prim
            "orientation": None,  # with respect to its parent prim or the world frame dependis if position or traslation is set
            "render_product_path": "/default_render_product",
        }
        camera_params_diff = [
            {
                "prim_path": "/World/fixed_camera/front",
                "name": "front",
                "translation": [1.0, 0.0, 0.0],
                "orientation": [1.0, 0.0, 0.0, 0.0],  #  (w, x, y, z)
            },
            {
                "prim_path": "/World/fixed_camera/down",
                "name": "down",
                "translation": [-1.0, 0.0, 0.0],
                "orientation": [1.0, 0.0, 0.0, 0.0],  #  (w, x, y, z)
            },
        ]
        self._fixed_cameras = []
        # composite the camera configuration
        for camera_param in camera_params_diff:
            # merge the common and specific parameters where may has the same key
            camera_params = {**camera_params_common, **camera_param}
            # create camera Prim with the help of the Camera class
            self._fixed_cameras.append(Camera(**camera_params))

        # Create camera graph for the existing Camera prim on the Iris model, which can be found
        # at the prim path `/World/quadrotor/body/Camera`. The camera prim path is the local path from the vehicle's prim path
        # to the camera prim, to which this graph will be connected. All ROS2 topics published by this graph will have
        # namespace `quadrotor` and frame_id `Camera` followed by the selected camera types (`rgb`, `camera_info`).
        config_multirotor.graphs = [ROS1Camera(camera_prim_path="body/Camera")]

        # create graph for the fixed cameras
        self._fixed_cameras_graphs = []
        world_prim = XFormPrim("/World", name="World")  # get the root prim
        for camera in self._fixed_cameras:
            self._fixed_cameras_graphs.append(ROS1Camera(camera_prim_path=camera.prim_path))
            self._fixed_cameras_graphs[-1].initialize(
                world_prim
            )  # initialize the graph manually and put it on the World prim

        # create the clock graph, which will publish the simulation time in /clock topic
        self._clock_graph = ROS1Clock(
            prim_path="clock_pub", topic_name="clock", namespace="", graph_evaluator="execution"
        )
        self._clock_graph.initialize(world_prim)  # put the clock graph on the World prim and initialize it manually

        # add the odom graph to the multirotor configuration, as it will be initialized in the Multirotor class,
        # and publish the odometry of the quadrotor in /odom topic
        config_multirotor.graphs.append(
            ROS1Odom(
                prim_path="odom_pub",
                topic_name="odom",
                namespace="",
                chassis_frame_id="base_link",
                odom_frame_id="odom",
                tf_namespace="",
                tf_topic_name="tf",
                graph_evaluator="execution",
            )  # the odom graph will be put on the quadrotor Prim
        )

        Multirotor(
            "/World/quadrotor",
            ROBOTS["Iris"],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # create TF graph, which will publish the transformation tree of the quadrotor and the fixed cameras in /tf topic
        self._tf_graph = ROS1TF(
            prim_path="/tf_pub",
            tf_parent_prim_path="/World",
            tf_child_prims_path=[
                "/World/quadrotor",
                "/World/quadrotor/body/Camera",
                "/World/fixed_camera/front",
                "/World/fixed_camera/down",
            ],
            topic_name="tf",
            namespace="",
            graph_evaluator="execution",
        )
        self._tf_graph.initialize()  # initialize the TF graph manually, the TF graph Prim will be created in the root(/) Prim

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

#!/usr/bin/env python
"""
| File: python_control_backend.py
| Author: Marcelo Jacinto and Joao Pinto (marcelo.jacinto@tecnico.ulisboa.pt, joao.s.pinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to use the control backends API to create a custom controller 
for the vehicle from scratch and use it to perform a simulation, without using PX4 nor ROS.
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

# Used for adding extra lights to the environment
import isaacsim.core.utils.prims as prim_utils

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.vehicles.multirotor_batch import MultirotorBatch, MultirotorBatchConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Import the custom python control backend
import sys, os
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)) + '/utils')
from nonlinear_controller_batch import NonlinearControllerBatch

# Auxiliary scipy and numpy modules
import numpy as np
import torch
from pegasus.simulator.logic.transforms import euler_angles_to_matrix, matrix_to_quaternion


# Use pathlib for parsing the desired trajectory from a CSV file
from pathlib import Path

import random
from isaacsim.util.debug_draw import _debug_draw


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

        world_settings = dict(self.pg._world_settings) 
        world_settings["device"] = "cuda"           

        self.pg._world = World(**world_settings)

        self.world = self.pg.world

        # Add a custom light with a high-definition HDR surround environment of an exhibition hall,
        # instead of the typical ground plane
        prim_utils.create_prim(
            "/World/Light/DomeLight",
            "DomeLight",
            position=np.array([1.0, 1.0, 1.0]),
            attributes={
                "inputs:intensity": 5e3,
                "inputs:color": (1.0, 1.0, 1.0),
                "inputs:texture:file": "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/NVIDIA/Assets/Skies/Indoor/ZetoCGcom_ExhibitionHall_Interior1.hdr"
                # Alternative sky: https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/NVIDIA/Assets/Skies/Cloudy/abandoned_parking_4k.hdr
            }
        )

        # Get the current directory used to read trajectories and save results
        self.curr_dir = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve())

        # Number of vehicles in the batch
        n_vehicles = 2

        # Create a single multirotor batch configuration
        config_multirotor = MultirotorBatchConfig(n_vehicles=n_vehicles)

        # Since the current batch controller uses a single trajectory for the whole batch,
        # choose one trajectory file for all vehicles in the batch.
        config_multirotor.backends = [
            NonlinearControllerBatch(
                trajectory_files=[
                    self.curr_dir + "/trajectories/pitch_relay_90_deg_1.csv",
                    self.curr_dir + "/trajectories/pitch_relay_90_deg_2.csv",
                ],
                results_files=[
                    self.curr_dir + "/results/statistics_batch_1.pt",
                    self.curr_dir + "/results/statistics_batch_2.pt"
                ],
                Ki=[0.5, 0.5, 0.5],
                Kr=[2.0, 2.0, 2.0],
                n_vehicles=n_vehicles,
                device=self.pg._world_settings["device"],
            )
        ]

        # Spawn the multirotor batch
        MultirotorBatch1 = MultirotorBatch(
            stage_prefix="/World/quadrotor",
            usd_file=ROBOTS["Iris"],
            vehicle_batch_id=1,
            n_vehicles=n_vehicles,
            init_pos=[
                [0.0, -1.5, 8.0],
                [2.3, -1.5, 8.0],
            ],
            init_orientation=[
                matrix_to_quaternion(euler_angles_to_matrix(torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32, device=self.pg._world_settings["device"]), "XYZ")),
                matrix_to_quaternion(euler_angles_to_matrix(torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32, device=self.pg._world_settings["device"]), "XYZ")),
            ],
            config=config_multirotor,
        )

        # Set the camera to a nice position so that we can see the 2 drones almost touching each other
        self.pg.set_viewport_camera([7.53, -1.6, 4.96], [0.0, 3.3, 7.0])

        # Read and draw a reference trajectory in Isaac Sim
        trajectory1 = np.flip(np.genfromtxt(self.curr_dir + "/trajectories/pitch_relay_90_deg_1.csv", delimiter=','), axis=0)
        num_samples1,_ = trajectory1.shape
        trajectory2 = np.flip(np.genfromtxt(self.curr_dir + "/trajectories/pitch_relay_90_deg_2.csv", delimiter=','), axis=0)
        num_samples2,_ = trajectory2.shape

        # Draw the lines of the desired trajectory in Isaac Sim with the same color as the output plots for the paper
        draw = _debug_draw.acquire_debug_draw_interface()
        point_list_1 = [(trajectory1[i,1], trajectory1[i,2], trajectory1[i,3]) for i in range(num_samples1)]
        draw.draw_lines_spline(point_list_1, (31/255, 119/255, 180/255, 1), 5, False)

        point_list_2 = [(trajectory2[i,1], trajectory2[i,2], trajectory2[i,3]) for i in range(num_samples2)]
        draw.draw_lines_spline(point_list_2, (255/255, 0, 0, 1), 5, False)

        self.world.reset()


    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running():

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
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
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
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

    def __init__(self, n_envs=8, spacing=3.0, device="cpu"):
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
        world_settings["device"] = device           

        self.pg._world = World(**world_settings)

        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

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
        self.n_vehicles = n_envs

        # Create a single multirotor batch configuration
        config_multirotor = MultirotorBatchConfig(n_vehicles=self.n_vehicles)

        # Since the current batch controller uses a single trajectory for the whole batch,
        # choose one trajectory file for all vehicles in the batch.
        config_multirotor.backends = []

        # Spawn the multirotor batch
        self.MultirotorBatch1 = MultirotorBatch(
            stage_prefix="/World/quadrotor",
            usd_file=ROBOTS["Iris"],
            vehicle_batch_id=1,
            n_vehicles=self.n_vehicles,
            config=config_multirotor,
            spacing=3.0
        )

        self.world.get_physics_context().set_gravity(0.0)
        self.world.reset()

        self.device = device


    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running():

            forces = torch.zeros((self.n_vehicles, self.MultirotorBatch1.parts_per_vehicle, 3), device=self.device, dtype=torch.float32)
            forces[:, 1:, 2] = float(0.005)

            torques = torch.zeros((self.n_vehicles, self.MultirotorBatch1.parts_per_vehicle, 3), device=self.device, dtype=torch.float32)
            torques[:, 0, 2] = float(0.01) 

            self.MultirotorBatch1.apply_forces_and_torques_all_parts(forces, torques)

            # Update the UI of the app and perform the physics step
            self.world.step()
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    pg_app = PegasusApp(n_envs=1, spacing=3.0, device="cuda")

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
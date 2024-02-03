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

# Used for adding extra lights to the environment
import omni.isaac.core.utils.prims as prim_utils

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Import the custom python control backend
from utils.nonlinear_controller import NonlinearController

# Auxiliary scipy and numpy modules
import numpy as np
from scipy.spatial.transform import Rotation

# Use os and pathlib for parsing the desired trajectory from a CSV file
import os
from pathlib import Path

import random
from omni.isaac.debug_draw import _debug_draw


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

        # Add a custom light with a high-definition HDR surround environment of an exhibition hall,
        # instead of the typical ground plane
        prim_utils.create_prim(
            "/World/Light/DomeLight",
            "DomeLight",
            position=np.array([1.0, 1.0, 1.0]),
            attributes={
                "inputs:intensity": 5e3,
                "inputs:color": (1.0, 1.0, 1.0),
                "inputs:texture:file": "omniverse://localhost/NVIDIA/Assets/Skies/Indoor/ZetoCGcom_ExhibitionHall_Interior1.hdr"
            }
        )

        # Get the current directory used to read trajectories and save results
        self.curr_dir = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve())

        # Create the vehicle 1
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor1 = MultirotorConfig()
        config_multirotor1.backends = [NonlinearController(
            trajectory_file=self.curr_dir + "/trajectories/pitch_relay_90_deg_1.csv",
            results_file=self.curr_dir + "/results/statistics_1.npz",
            Ki=[0.5, 0.5, 0.5],
            Kr=[2.0, 2.0, 2.0])]

        Multirotor(
            "/World/quadrotor1",
            ROBOTS['Iris'],
            1,
            [0,-1.5, 8.0],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor1,
        )

        # Create the vehicle 2
        #Try to spawn the selected robot in the world to the specified namespace
        config_multirotor2 = MultirotorConfig()
        config_multirotor2.backends = [NonlinearController(
            trajectory_file=self.curr_dir + "/trajectories/pitch_relay_90_deg_2.csv",
            results_file=self.curr_dir + "/results/statistics_2.npz",
            Ki=[0.5, 0.5, 0.5],
            Kr=[2.0, 2.0, 2.0])]

        Multirotor(
            "/World/quadrotor2",
            ROBOTS['Iris'],
            2,
            [2.3,-1.5, 8.0],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor2,
        )

        # Set the camera to a nice position so that we can see the 2 drones almost touching each other
        self.pg.set_viewport_camera([7.53, -1.6, 4.96], [0.0, 3.3, 7.0])

        # Read the trajectories and plot them inside isaac sim
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
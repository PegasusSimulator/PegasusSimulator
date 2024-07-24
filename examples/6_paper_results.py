#!/usr/bin/env python
"""
| File: python_control_backend.py
| Author: Marcelo Jacinto and Joao Pinto (marcelo.jacinto@tecnico.ulisboa.pt, joao.s.pinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to use the control backends API to create a custom controller 
for the vehicle from scratch and use it to perform a simulation, without using PX4 nor ROS. NOTE: to see the HDR
environment as shown in the video and paper, you must have opened ISAAC SIM at least once thorugh the OMNIVERSE APP,
otherwise, the path to the HDR environment is not recognized.
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
import omni.isaac.core.utils.prims as prim_utils

import omni.kit.commands
from pxr import Sdf

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.dynamics.linear_drag import LinearDrag
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

import sys, os
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)) + '/utils')
from nonlinear_controller import NonlinearController

# Auxiliary scipy and numpy modules
import numpy as np
from scipy.spatial.transform import Rotation

# Use pathlib for parsing the desired trajectory from a CSV file
from pathlib import Path

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
        self.pg._world_settings = {"physics_dt": 1.0 / 500.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

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
        config_multirotor1.drag = LinearDrag([0.0, 0.0, 0.0])

        # Use the nonlinear controller with the built-in exponential trajectory
        config_multirotor1.backends = [NonlinearController(
            trajectory_file=None,
            results_file=self.curr_dir + "/results/statistics_1.npz")]

        Multirotor(
            "/World/quadrotor1",
            ROBOTS['Iris'],
            1,
            [-5.0,0.00,1.00],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor1,
        )

        # Create the vehicle 2
        #Try to spawn the selected robot in the world to the specified namespace
        config_multirotor2 = MultirotorConfig()

        # Use the nonlinear controller with the built-in exponential trajectory
        config_multirotor2.backends = [NonlinearController(
            trajectory_file=None,
            results_file=self.curr_dir + "/results/statistics_2.npz",
            reverse=True)]

        Multirotor(
            "/World/quadrotor2",
            ROBOTS['Iris'],
            2,
            [-5.0,4.5,1.0],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor2,
        )

        # Set the camera to a nice position so that we can see the 2 drones almost touching each other
        self.pg.set_viewport_camera([1.0, 5.15, 1.65], [0.0, -1.65, 3.3])

        # Draw the lines of the desired trajectory in Isaac Sim with the same color as the output plots for the paper
        gamma = np.arange(start=-5.0, stop=5.0, step=0.01)
        num_samples = gamma.size
        trajectory1 = [config_multirotor1.backends[0].pd(gamma[i], 0.6) for i in range(num_samples)]
        trajectory2 = [config_multirotor2.backends[0].pd(gamma[i], 0.6, reverse=True) for i in range(num_samples)]

        draw = _debug_draw.acquire_debug_draw_interface()
        point_list_1 = [(trajectory1[i][0], trajectory1[i][1], trajectory1[i][2]) for i in range(num_samples)]
        draw.draw_lines_spline(point_list_1, (31/255, 119/255, 180/255, 1), 5, False)

        point_list_2 = [(trajectory2[i][0], trajectory2[i][1], trajectory2[i][2]) for i in range(num_samples)]
        draw.draw_lines_spline(point_list_2, (255/255, 0, 0, 1), 5, False)

        # Reset the world
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

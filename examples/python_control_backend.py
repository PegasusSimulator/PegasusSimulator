#!/usr/bin/env python
"""
| File: python_control_backend.py
| Author: Marcelo Jacinto and Joao Pinto (marcelo.jacinto@tecnico.ulisboa.pt, joao.s.pinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to use the control backends API to create a custom controller for the vehicle from scratch and use it to perform a simulation, without using PX4 nor ROS.
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

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Auxiliary scipy and numpy modules
import numpy as np       
from scipy.spatial.transform import Rotation

# Use os, pandas and pathlib for parsing the desired trajectory from a CSV file
import os
import pandas as pd
from pathlib import Path

class NonlinearController(Backend):
    """A nonlinear controller class. It implements a nonlinear controller that allows a vehicle to track
    aggressive trajectories. This controlers is well described in the paper
    
    [1] J. Pinto, B. J. Guerreiro and R. Cunha, "Planning Parcel Relay Manoeuvres for Quadrotors," 
    2021 International Conference on Unmanned Aircraft Systems (ICUAS), Athens, Greece, 2021, 
    pp. 137-145, doi: 10.1109/ICUAS51884.2021.9476757.
    """

    def __init__(self):

        # The current state of the vehicle expressed in the inertial frame (in ENU)
        self.p = np.zeros((3,))         # The vehicle position
        self.R: Rotation = Rotation.identity()    # The vehicle attitude
        self.w = np.zeros((3,))         # The angular velocity of the vehicle
        self.v = np.zeros((3,))         # The linear velocity of the vehicle in the inertial frame

        # The trajectory references for the controller to track, i.e.
        # the target positions [m], velocity [m/s], accelerations [m/s^2], jerk [m/s^3], yaw-angle [rad], yaw-rate [rad/s]
        self.p_ref = np.array([0.0, 0.0, 1.0])
        self.v_ref = np.zeros((3,))
        self.a_ref = np.zeros((3,))
        self.j_ref = np.zeros((3,))
        self.yaw_ref = 0.0
        self.yaw_rate_ref = 0.0

        # Define the control gains matrix for the outer-loop
        self.Kp = np.diag([12.0, 12.0, 12.0])
        self.Kd = np.diag([9.0, 9.0, 9.0])

        # Define the dynamic parameters for the vehicle
        self.m = 1.5        # Mass in Kg
        self.g = 9.81       # The gravity acceleration ms^-2

        # Read the target trajectory from a CSV file inside the trajectories directory
        trajectory = self.read_trajectory_from_csv("slow_xy_ellipse.csv")

    def read_trajectory_from_csv(self, file_name: str):
        """Auxiliar method used to read the desired trajectory from a CSV file

        Args:
            file_name (str): A string with the name of the trajectory inside the trajectories directory

        Returns:
            pd.DataFrame: A Pandas dataframe table with the target trajectory to be executed over time
        """
        
        # Get the complete path to the CSV file
        csv_trajectory = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve()) + "/trajectories/" + file_name

        # Read the trajectory to a pandas frame
        return pd.read_csv(csv_trajectory)


    def start(self):
        pass

    def stop(self):
        pass

    def update_sensor(self, sensor_type: str, data):
        """
        Do nothing. For now ignore all the sensor data and just use the state directly for demonstration purposes. 
        This is a callback that is called at every physics step.

        Args:
            sensor_type (str): The name of the sensor providing the data
            data (dict): A dictionary that contains the data produced by the sensor
        """
        pass

    def update_state(self, state: State):
        """
        Method that updates the current state of the vehicle. This is a callback that is called at every physics step

        Args:
            state (State): The current state of the vehicle.
        """
        self.p = state.position
        self.R = Rotation.from_quat(state.attitude)
        self.w = state.angular_velocity
        self.v = state.linear_velocity

    def input_reference(self):
        """
        Method that is used to return the latest target angular velocities to be applied to the vehicle

        Returns:
            A list with the target angular velocities for each individual rotor of the vehicle
        """
        return [0.0, 0.0, 0.0, 0.0]

    def update(self, dt: float):
        """Method that implements the nonlinear control law and updates the target angular velocities for each rotor. 
        This method will be called by the simulation on every physics step

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Compute the tracking errors
        ep = self.p - self.p_ref
        ev = self.v - self.v_ref

        # Compute F_des term
        F_des = -(self.Kp @ ep) -(self.Kd @ ev) + np.array([0.0, 0.0, self.m * self.g]) + (self.m * self.a_ref)

        # Get the current axis Z_B (given by the last column of the rotation matrix)
        Z_B = self.R.as_matrix()[:,2]

        # Get the desired total thrust in Z_B direction (u_1)
        u_1 = F_des @ Z_B

        # Compute the desired body-frame axis Z_b
        Z_b_des = F_des / np.linalg.norm(F_des)

        # Compute X_C_des 
        X_c_des = np.array([np.cos(self.yaw_ref), np.sin(self.yaw_ref), 0.0])

        # Compute Y_b_des
        Z_b_cross_X_c = np.cross(Z_b_des, X_c_des)
        Y_b_des = Z_b_cross_X_c / np.linalg.norm(Z_b_cross_X_c)

        # Compute X_b_des
        X_b_des = np.cross(Y_b_des, Z_b_des)

        # Compute the desired rotation R_des = [X_b_des | Y_b_des | Z_b_des]
        R_des = Rotation.from_matrix(np.array([X_b_des, Y_b_des, Z_b_des]).T)

        # Compute the rotation error
        e_R = 0.5 * self.vee((R_des.inv() * self.R) - (self.R.inv() * R_des))

        # Compute the desired angular velocity by projecting the angular velocity in the Xb-Yb plane


        # Compute the angular velocity error
        e_w = self.w - w_des
    
    def apply_force_and_torques(self, force: float, torque: np.ndarray):
        """Method that given the total force [N] and the torque vector [\tau_x, \tau_y, \tau_z]^T [Nm]
        computes the actual angular velocities to apply to each rotor of the vehicle.

        Args:
            force (float): The total force [N] to apply to the vehicle body frame
            torque (np.ndarray): The total torque vector [\tau_x, \tau_y, \tau_z]^T [Nm] to apply to the vehicle
        """
        pass

    @staticmethod
    def vee(S):
        """Auxiliary function that computes the 'v' map which takes elements from so(3) to R^3.

        Args:
            S (np.array): A matrix in so(3)
        """
        return np.array([-S[1,2], S[0,2], -S[0,1]])


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
        config_multirotor.backends = [NonlinearController()]

        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Create a timeline callback to close the app when the simulation stops
        self.world.add_timeline_callback('template_timeline_callback', self.timeline_callback)

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def timeline_callback(self, timeline_event):
        """An example timeline callback. It will get invoked every time a timeline event occurs. In this example,
        we will check if the event is for a 'simulation stop'. If so, we will attempt to close the app

        Args:
            timeline_event: A timeline event
        """
        if self.world.is_stopped():
            self.stop_sim = True

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

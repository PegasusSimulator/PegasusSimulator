#!/usr/bin/env python
"""
This example file demonstrates how to create custom backends that allow for controling and receiving simulation data from
the Pegasus simulator without necessarily using PX4 or ROS2
"""

# Imports to start Isaac Sim from this script
from omni.isaac.kit import SimulationApp

# Start Isaac Sim's simulation environment
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
import carb

# Import the pegasus API
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.pegasus_simulator import PegasusSimulator
from pegasus.simulator.logic.vehicles import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends import Backend, MavlinkBackend, MavlinkBackendConfig

# Auxiliar numpy and Scipy imports
import numpy as np
from scipy.spatial.transform import Rotation

from omni.isaac.core.objects import DynamicCuboid

class NonlinearControlBackend(Backend):
    """
    A nonlinear control backend used to track aggressive trajectories as described in the paper
    """
     
    def __init__(self):
        self.p = np.zeros((3,))         # The vehicle position
        self.R = Rotation.identity()    # The vehicle attitude
        self.w = np.zeros((3,))         # The angular velocity of the vehicle
        self.v = np.zeros((3,))         # The linear velocity of the vehicle in the inertial frame

        # Define the target velocities, positions and accelerations
        self.p_ref = np.zeros((3,))
        self.v_ref = np.zeros((3,))
        self.a_ref = np.zeros((3,))

        # Define the target yaw reference in radians
        self.yaw_ref = 0.0

        # Define the control gains matrix
        self.Kp = np.diag([3.0, 3.0, 3.0])
        self.Kd = np.diag([3.0, 3.0, 3.0])

        # Define the dynamic parameters
        self.m = 1.5        # Mass in Kg
        self.g = 9.81       # The gravity acceleration ms^-2

        # Allocation matrix of the Iris drone
        #self.alocation = np.array([[1.0, 1.0, 1.0, 1.0],
        #                           [],
        #                           [],
        #                           []])

    def update_sensor(self, sensor_type: str, data):
        """
        Do nothing. For now ignore all the sensor data and just use the state directly for demonstration purposes. 
        This is a callback that is called at every physics step

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
        self.p = state.get_position_ned()
        self.R = Rotation.from_quat(state.get_attitude_ned_frd())
        self.w = state.get_angular_velocity_frd()
        self.v = state.get_linear_velocity_ned()

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

        carb.log_warn("Running")

        # Compute the tracking errors
        ep = self.p - self.p_ref
        ev = self.v - self.v_ref

        # Compute F_des term
        F_des = -(self.Kp @ ep) -(self.Kd @ ev) + np.array([0.0, 0.0, self.m * self.g]) + self.a_ref

        # Compute u1 - TODO

        # Compute the desired body-frame axis Z_b
        Z_b_des = F_des / np.linalg.norm(F_des)

        # Compute the desired orientation - TODO
        R_des = Rotation.identity()

        # Compute the rotation error - TODO
        #e_R = 0.5 * S((R_des.inv() * self.R) - (self.R.inv() * R_des))

        # Compute the angular velocity error
        #e_w = self.w - self.w_des


    def start(self):
        pass

    def stop(self):
        pass

def test():

    carb.log_warn("Testing")

def main():
    """
    The code where the simulation will actually get executed
    """

    # Start the Pegasus Simulator backend
    pg_sim = PegasusSimulator()    

    # Load the simulation world
    pg_sim.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])

    # Create the multirotor configuration using the new custom control backend. Not using mavlink nor ROS for this one. 
    # Just plain python
    control_backend = NonlinearControlBackend()
    config_multirotor = MultirotorConfig()
    config_multirotor.backends = [control_backend]
    

    # Try to spawn the selected robot in the world to the specified namespace
    pos = [0.0, 0.0, 0.1] # The initial position of the vehicle in ENU
    att = [0.0, 0.0, 0.0] # The initial orientation of the vehicle in ENU
    
    m = Multirotor("/World/quadrotor", ROBOTS["Iris"], 0, pg_sim.world, pos, Rotation.from_euler("XYZ", att, degrees=True).as_quat(), config=config_multirotor)
    
    # episode counter
    sim_time = 0.0
    sim_dt = pg_sim.world.get_physics_dt()
    carb.log_warn("SIM dt: " + str(sim_dt))

    pg_sim.world.add_physics_callback("test", test)
    
    # ----------------
    # Run the simulation
    # ----------------
    pg_sim.world.reset()
    #world.play()
    while simulation_app.is_running():

        # update sim-time
        sim_time += sim_dt

        carb.log_warn("SIM dt: " + str(sim_dt))

        # Perform another physics simulation step and render
        # we have control over stepping physics and rendering in this workflow runs in sync
        pg_sim.world.step(render=True)
    
    # close Isaac Sim
    simulation_app.close() 

    


if __name__ == '__main__':

    # Execute the main simulation code
    main()
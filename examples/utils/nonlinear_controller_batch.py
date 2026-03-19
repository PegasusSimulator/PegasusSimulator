#!/usr/bin/env python
"""
| File: nonlinear_controller.py
| Author: Marcelo Jacinto and Joao Pinto (marcelo.jacinto@tecnico.ulisboa.pt, joao.s.pinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to use the control backends API to create a custom controller 
for the vehicle from scratch and use it to perform a simulation, without using PX4 nor ROS. In this controller, we
provide a quick way of following a given trajectory specified in csv files or track an hard-coded trajectory based
on exponentials! NOTE: This is just an example, to demonstrate the potential of the API. A much more flexible solution
can be achieved
"""

# Imports to be able to log to the terminal with fancy colors
import carb

# Imports from the Pegasus library
from pegasus.simulator.logic.state_batch import StateBatch as State
from pegasus.simulator.logic.backends import Backend

# Auxiliary scipy and numpy modules
import numpy as np
import torch

from typing import Sequence

#from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.transforms import quaternion_to_matrix


class NonlinearControllerBatch(Backend):
    """A nonlinear controller class. It implements a nonlinear controller that allows a vehicle to track
    aggressive trajectories. This controlers is well described in the papers
    
    [1] J. Pinto, B. J. Guerreiro and R. Cunha, "Planning Parcel Relay Manoeuvres for Quadrotors," 
    2021 International Conference on Unmanned Aircraft Systems (ICUAS), Athens, Greece, 2021, 
    pp. 137-145, doi: 10.1109/ICUAS51884.2021.9476757.
    [2] D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," 
    2011 IEEE International Conference on Robotics and Automation, Shanghai, China, 2011, 
    pp. 2520-2525, doi: 10.1109/ICRA.2011.5980409.
    """

    def __init__(self, 
        trajectory_files: Sequence[str] = None, 
        results_files: Sequence[str]=None, 
        reverse=False, 
        Kp=[10.0, 10.0, 10.0],
        Kd=[8.5, 8.5, 8.5],
        Ki=[1.50, 1.50, 1.50],
        Kr=[3.5, 3.5, 3.5],
        Kw=[0.5, 0.5, 0.5],
        n_vehicles=1,
        device = "cpu"
    ):

        # Set device
        self.device = device

        # The current rotor references [rad/s]
        self.input_ref = torch.zeros((n_vehicles, 4), dtype=torch.float32, device=device)

        # The current state of the vehicle expressed in the inertial frame (in ENU)
        self.p = torch.zeros((n_vehicles, 3), dtype=torch.float32, device=device)                   # The vehicle position
        self.R = torch.eye(3, dtype=torch.float32, device=device).unsqueeze(0).repeat(n_vehicles, 1, 1)        
        self.w = torch.zeros((n_vehicles, 3), dtype=torch.float32, device=device)                   # The angular velocity of the vehicle
        self.v = torch.zeros((n_vehicles, 3), dtype=torch.float32, device=device)                   # The linear velocity of the vehicle in the inertial frame
        self.a = torch.zeros((n_vehicles, 3), dtype=torch.float32, device=device)                   # The linear acceleration of the vehicle in the inertial frame

        # Define the control gains matrix for the outer-loop
        self.Kp = torch.diag(torch.tensor(Kp, dtype=torch.float32, device=device))
        self.Kd = torch.diag(torch.tensor(Kd, dtype=torch.float32, device=device))
        self.Ki = torch.diag(torch.tensor(Ki, dtype=torch.float32, device=device))
        self.Kr = torch.diag(torch.tensor(Kr, dtype=torch.float32, device=device))
        self.Kw = torch.diag(torch.tensor(Kw, dtype=torch.float32, device=device))

        self.int = torch.zeros((n_vehicles, 3), dtype=torch.float32, device=device)

        # Define the dynamic parameters for the vehicle
        self.m = 1.50        # Mass in Kg
        self.g = 9.81       # The gravity acceleration ms^-2

        # Read the target trajectory from a CSV file inside the trajectories directory
        # if a trajectory is provided. Otherwise, just perform the hard-coded trajectory provided with this controller
        self.indices = torch.zeros(n_vehicles, dtype=torch.long, device=device)

        if trajectory_files is not None:
            if len(trajectory_files) != n_vehicles:
                raise ValueError(f"trajectory_files must have length {n_vehicles}, got {len(trajectory_files)}")

            self.trajectories = [self.read_trajectory_from_csv(f) for f in trajectory_files]
            self.max_indices = torch.tensor(
                [traj.shape[0] for traj in self.trajectories],
                dtype=torch.long,
                device=device
            )
            self.total_time = 0.0
        # Use the built-in trajectory hard-coded for this controller
        else:
            # Set the initial time for starting when using the built-in trajectory (the time is also used in this case
            # as the parametric value)
            self.total_time = -5.0
            # Signal that we will not used a received trajectory
            self.trajectories = None
            self.max_indices = torch.ones(n_vehicles, dtype=torch.long, device=device)

        self.reverse = reverse

        # Auxiliar variable, so that we only start sending motor commands once we get the state of the vehicle
        self.reveived_first_state = False

        # Lists used for analysing performance statistics
        if results_files is not None and len(results_files) != n_vehicles:
            raise ValueError(f"results_files must have length {n_vehicles}, got {len(results_files)}")

        self.results_files = results_files

        self.time_vector = []
        self.desired_position_over_time = []
        self.position_over_time = []
        self.position_error_over_time = []
        self.velocity_error_over_time = []
        self.atittude_error_over_time = []
        self.attitude_rate_error_over_time = []

    def read_trajectory_from_csv(self, file_name: str):
        """Auxiliar method used to read the desired trajectory from a CSV file

        Args:
            file_name (str): A string with the name of the trajectory inside the trajectories directory

        Returns:
            np.ndarray: A numpy matrix with the trajectory desired states over time
        """

        # Read the trajectory to a pandas frame
        return np.flip(np.genfromtxt(file_name, delimiter=','), axis=0)


    def start(self):
        """
        Reset the control and trajectory index
        """
        self.reset_statistics()
        

    def stop(self):
        """
        Stopping the controller. Saving the statistics data for plotting later
        """

        # Check if we should save the statistics to some file or not
        if self.results_files is None:
            return

        if len(self.time_vector) == 0:
            carb.log_warn("No statistics to save.")
            self.reset_statistics()
            return
        
        time_tensor = torch.tensor(self.time_vector, dtype=torch.float32, device=self.device)

        p_hist = torch.stack(self.position_over_time)                  # (T, N, 3)
        desired_p_hist = torch.stack(self.desired_position_over_time)  # (T, N, 3)
        ep_hist = torch.stack(self.position_error_over_time)           # (T, N, 3)
        ev_hist = torch.stack(self.velocity_error_over_time)           # (T, N, 3)
        er_hist = torch.stack(self.atittude_error_over_time)           # (T, N, 3)
        ew_hist = torch.stack(self.attitude_rate_error_over_time)      # (T, N, 3)

        n_vehicles = p_hist.shape[1]

        for i in range(n_vehicles):
            statistics = {}
            statistics["time"] = time_tensor.clone()
            statistics["p"] = p_hist[:, i, :]
            statistics["desired_p"] = desired_p_hist[:, i, :]
            statistics["ep"] = ep_hist[:, i, :]
            statistics["ev"] = ev_hist[:, i, :]
            statistics["er"] = er_hist[:, i, :]
            statistics["ew"] = ew_hist[:, i, :]

            torch.save(statistics, self.results_files[i])
            carb.log_warn("Statistics saved to: " + self.results_files[i])

        self.reset_statistics()

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
        self.R = quaternion_to_matrix(state.attitude)
        self.w = state.angular_velocity
        self.v = state.linear_velocity

        self.reveived_first_state = True

    def input_reference(self):
        """
        Method that is used to return the latest target angular velocities to be applied to the vehicle

        Returns:
            A list with the target angular velocities for each individual rotor of the vehicle
        """
        return self.input_ref

    def update(self, dt: float):
        """Method that implements the nonlinear control law and updates the target angular velocities for each rotor. 
        This method will be called by the simulation on every physics step

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        
        if self.reveived_first_state == False:
            return

        # -------------------------------------------------
        # Update the references for the controller to track
        # -------------------------------------------------
        self.total_time += dt
        
        # Update using an external trajectory
        if self.trajectories is not None:
            # the target positions [m], velocity [m/s], accelerations [m/s^2], jerk [m/s^3], yaw-angle [rad], yaw-rate [rad/s]
            p_ref = torch.zeros((self.p.shape[0], 3), dtype=torch.float32, device=self.device)
            v_ref = torch.zeros((self.p.shape[0], 3), dtype=torch.float32, device=self.device)
            a_ref = torch.zeros((self.p.shape[0], 3), dtype=torch.float32, device=self.device)
            j_ref = torch.zeros((self.p.shape[0], 3), dtype=torch.float32, device=self.device)
            yaw_ref = torch.zeros((self.p.shape[0],), dtype=torch.float32, device=self.device)
            yaw_rate_ref = torch.zeros((self.p.shape[0],), dtype=torch.float32, device=self.device)

            for i in range(self.p.shape[0]):
                idx = int(self.indices[i].item())
                max_idx = int(self.max_indices[i].item())
                traj = self.trajectories[i]

                # Check if we need to update to the next trajectory index
                if idx < max_idx - 1 and self.total_time >= traj[idx + 1, 0]:
                    idx += 1
                    self.indices[i] = idx

                p_ref[i] = torch.tensor(traj[idx, 1:4], dtype=torch.float32, device=self.device)
                v_ref[i] = torch.tensor(traj[idx, 4:7], dtype=torch.float32, device=self.device)
                a_ref[i] = torch.tensor(traj[idx, 7:10], dtype=torch.float32, device=self.device)
                j_ref[i] = torch.tensor(traj[idx, 10:13], dtype=torch.float32, device=self.device)
                yaw_ref[i] = torch.tensor(traj[idx, 13], dtype=torch.float32, device=self.device)
                yaw_rate_ref[i] = torch.tensor(traj[idx, 14], dtype=torch.float32, device=self.device)
        # Or update the reference using the built-in trajectory
        else:
            s = 0.6
            p_ref = self.pd(self.total_time, s, self.reverse).unsqueeze(0).expand(self.p.shape[0], -1)
            v_ref = self.d_pd(self.total_time, s, self.reverse).unsqueeze(0).expand(self.p.shape[0], -1)
            a_ref = self.dd_pd(self.total_time, s, self.reverse).unsqueeze(0).expand(self.p.shape[0], -1)
            j_ref = self.ddd_pd(self.total_time, s, self.reverse).unsqueeze(0).expand(self.p.shape[0], -1)
            yaw_ref = torch.as_tensor(self.yaw_d(self.total_time, s), dtype=torch.float32, device=self.device).repeat(self.p.shape[0])
            yaw_rate_ref = torch.as_tensor(self.d_yaw_d(self.total_time, s), dtype=torch.float32, device=self.device).repeat(self.p.shape[0])

        # -------------------------------------------------
        # Start the controller implementation
        # -------------------------------------------------

        # Compute the tracking errors
        ep = self.p - p_ref
        ev = self.v - v_ref
        self.int = self.int +  (ep * dt)
        ei = self.int

        # Compute F_des term
        F_des = -(ep @ self.Kp.T) - (ev @ self.Kd.T) - (ei @ self.Ki.T) + torch.tensor([0.0, 0.0, self.m * self.g], dtype=torch.float32, device=self.device).unsqueeze(0) + self.m * a_ref
        
        # Get the current axis Z_B (given by the last column of the rotation matrix)
        Z_B = self.R[:, :,2]

        # Get the desired total thrust in Z_B direction (u_1)
        u_1 = torch.sum(F_des * Z_B, dim=1)

        # Compute the desired body-frame axis Z_b
        Z_b_des = F_des / torch.linalg.norm(F_des, dim=1, keepdim=True)

        yaw_ref = torch.as_tensor(yaw_ref, dtype=torch.float32, device=self.device)
        yaw_rate_ref = torch.as_tensor(yaw_rate_ref, dtype=torch.float32, device=self.device)

        # Compute X_C_des 
        X_c_des = torch.stack((torch.cos(yaw_ref), torch.sin(yaw_ref), torch.zeros_like(yaw_ref),), dim=1)

        # Compute Y_b_des
        Z_b_cross_X_c = torch.cross(Z_b_des, X_c_des, dim=1)
        Y_b_des = Z_b_cross_X_c / torch.linalg.norm(Z_b_cross_X_c, dim=1, keepdim=True)

        # Compute X_b_des
        X_b_des = torch.cross(Y_b_des, Z_b_des, dim=1)

        # Compute the desired rotation R_des = [X_b_des | Y_b_des | Z_b_des]
        R_des = torch.stack((X_b_des, Y_b_des, Z_b_des), dim=2)

        # Compute the rotation error
        e_R_mat = torch.matmul(R_des.transpose(1, 2), self.R) - torch.matmul(self.R.transpose(1, 2), R_des)
        e_R = 0.5 * self.vee_batch(e_R_mat)

        # Compute an approximation of the current vehicle acceleration in the inertial frame (since we cannot measure it directly)
        self.a = (u_1.unsqueeze(1) * Z_B) / self.m - torch.tensor([0.0, 0.0, self.g], dtype=torch.float32, device=self.device).unsqueeze(0)
        
        # Compute the desired angular velocity by projecting the angular velocity in the Xb-Yb plane
        # projection of angular velocity on xB − yB plane
        # see eqn (7) from [2].
        proj = torch.sum(Z_b_des * j_ref, dim=1, keepdim=True)
        hw = (self.m / torch.clamp(u_1, min=1e-6)).unsqueeze(1) * (j_ref - proj * Z_b_des)
                
        # desired angular velocity
        w_des = torch.stack((-torch.sum(hw * Y_b_des, dim=1), torch.sum(hw * X_b_des, dim=1), yaw_rate_ref * Z_b_des[:, 2]), dim=1)
        
        # Compute the angular velocity error
        e_w = self.w - w_des

        # Compute the torques to apply on the rigid body
        tau = -(e_R @ self.Kr.T) - (e_w @ self.Kw.T)

        # Use the allocation matrix provided by the Multirotor vehicle to convert the desired force and torque
        # to angular velocity [rad/s] references to give to each rotor
        if self.vehicle:
            self.input_ref = self.vehicle.force_and_torques_to_velocities(u_1, tau)

        # ----------------------------
        # Statistics to save for later
        # ----------------------------
        self.time_vector.append(self.total_time)
        self.position_over_time.append(self.p)
        self.desired_position_over_time.append(p_ref)
        self.position_error_over_time.append(ep)
        self.velocity_error_over_time.append(ev)
        self.atittude_error_over_time.append(e_R)
        self.attitude_rate_error_over_time.append(e_w)

    @staticmethod
    def vee_batch(S):
        """Auxiliary function that computes the 'v' map which takes elements from so(3) to R^3.

        Args:
            S (torch.Tensor): A batch of matrices in so(3).
        """
        return torch.stack((-S[:, 1, 2], S[:, 0, 2], -S[:, 0, 1]), dim=1)
    
    def reset_statistics(self):

        self.indices.zero_()
        # If we received an external trajectory, reset the time to 0.0
        if self.trajectories is not None:
            self.total_time = 0.0
        # if using the internal trajectory, make the parametric value start at -5.0
        else:
            self.total_time = -5.0

        # Reset the lists used for analysing performance statistics
        self.time_vector = []
        self.desired_position_over_time = []
        self.position_over_time = []
        self.position_error_over_time = []
        self.velocity_error_over_time = []
        self.atittude_error_over_time = []
        self.attitude_rate_error_over_time = []

    # ---------------------------------------------------
    # Definition of an exponential trajectory for example
    # This can be used as a reference if not trajectory file is passed
    # as an argument to the constructor of this class
    # ---------------------------------------------------

    def pd(self, t, s, reverse=False):
        """The desired position of the built-in trajectory

        Args:
            t (float): The parametric value that guides the equation
            s (float): How steep and agressive the curve is
            reverse (bool, optional): Choose whether we want to flip the curve (so that we can have 2 drones almost touching). Defaults to False.

        Returns:
            torch.tensor: A 3x1 tensor with the x, y ,z desired [m]
        """
        t = torch.as_tensor(t, dtype=torch.float32, device=self.device)
        s = torch.as_tensor(s, dtype=torch.float32, device=self.device)

        x = t
        z = 1 / s * torch.exp(-0.5 * torch.power(t/s, 2)) + 1.0
        y = 1 / s * torch.exp(-0.5 * torch.power(t/s, 2))

        if reverse == True:
            y = -1 / s * torch.exp(-0.5 * torch.power(t/s, 2)) + 4.5

        return torch.tensor([x,y,z], dtype=torch.float32, device=self.device)

    def d_pd(self, t, s, reverse=False):
        """The desired velocity of the built-in trajectory

        Args:
            t (float): The parametric value that guides the equation
            s (float): How steep and agressive the curve is
            reverse (bool, optional): Choose whether we want to flip the curve (so that we can have 2 drones almost touching). Defaults to False.

        Returns:
            torch.tensor: A 3x1 tensor with the d_x, d_y ,d_z desired [m/s]
        """
        t = torch.as_tensor(t, dtype=torch.float32, device=self.device)
        s = torch.as_tensor(s, dtype=torch.float32, device=self.device)

        x = 1.0
        y = -(t * torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,3)
        z = -(t * torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,3)

        if reverse == True:
            y = (t * torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,3)

        return torch.tensor([x,y,z], dtype=torch.float32, device=self.device)

    def dd_pd(self, t, s, reverse=False):
        """The desired acceleration of the built-in trajectory

        Args:
            t (float): The parametric value that guides the equation
            s (float): How steep and agressive the curve is
            reverse (bool, optional): Choose whether we want to flip the curve (so that we can have 2 drones almost touching). Defaults to False.

        Returns:
            np.ndarray: A 3x1 array with the dd_x, dd_y ,dd_z desired [m/s^2]
        """
        t = torch.as_tensor(t, dtype=torch.float32, device=self.device)
        s = torch.as_tensor(s, dtype=torch.float32, device=self.device)

        x = 0.0
        y = (torch.power(t,2)*torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,5) - torch.exp(-torch.power(t,2)/(2*torch.power(s,2)))/torch.power(s,3)
        z = (torch.power(t,2)*torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,5) - torch.exp(-torch.power(t,2)/(2*torch.power(s,2)))/torch.power(s,3)

        if reverse == True:
            y = torch.exp(-torch.power(t,2)/(2*torch.power(s,2)))/torch.power(s,3) - (torch.power(t,2)*torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,5)

        return torch.tensor([x,y,z], dtype=torch.float32, device=self.device)

    def ddd_pd(self, t, s, reverse=False):
        """The desired jerk of the built-in trajectory

        Args:
            t (float): The parametric value that guides the equation
            s (float): How steep and agressive the curve is
            reverse (bool, optional): Choose whether we want to flip the curve (so that we can have 2 drones almost touching). Defaults to False.

        Returns:
            torch.tensor: A 3x1 tensor with the ddd_x, ddd_y ,ddd_z desired [m/s^3]
        """
        t = torch.as_tensor(t, dtype=torch.float32, device=self.device)
        s = torch.as_tensor(s, dtype=torch.float32, device=self.device)

        x = 0.0
        y = (3*t*torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,5) - (torch.power(t,3)*torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,7)
        z = (3*t*torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,5) - (torch.power(t,3)*torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,7)

        if reverse == True:
            y = (torch.power(t,3)*torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,7) - (3*t*torch.exp(-torch.power(t,2)/(2*torch.power(s,2))))/torch.power(s,5)

        return torch.tensor([x,y,z], dtype=torch.float32, device=self.device)

    def yaw_d(self, t, s):
        """The desired yaw of the built-in trajectory

        Args:
            t (float): The parametric value that guides the equation
            s (float): How steep and agressive the curve is
            reverse (bool, optional): Choose whether we want to flip the curve (so that we can have 2 drones almost touching). Defaults to False.

        Returns:
            torch.tensor: A float with the desired yaw in rad
        """
        return torch.tensor(0.0, dtype=torch.float32, device=self.device)
    
    def d_yaw_d(self, t, s):
        """The desired yaw_rate of the built-in trajectory

        Args:
            t (float): The parametric value that guides the equation
            s (float): How steep and agressive the curve is
            reverse (bool, optional): Choose whether we want to flip the curve (so that we can have 2 drones almost touching). Defaults to False.

        Returns:
            torch.tensor: A float with the desired yaw_rate in rad/s
        """
        return torch.tensor(0.0, dtype=torch.float32, device=self.device)

    def reset(self):
        """
        Method that when implemented, should handle the reset of the vehicle simulation to its original state
        """
        pass

    def update_graphical_sensor(self, sensor_type: str, data):
        """
        For this demo we do not care about graphical sensors such as camera, therefore we can ignore this callback
        """
        pass
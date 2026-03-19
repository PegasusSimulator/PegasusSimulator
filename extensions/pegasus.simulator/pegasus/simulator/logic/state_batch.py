"""
| File: state.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Describes the state of a vehicle (or rigidbody).
"""
__all__ = ["StateBatch"]

import torch
from pegasus.simulator.logic.rotations import rot_ENU_to_NED, rot_FLU_to_FRD
from pegasus.simulator.logic.transforms import quaternion_to_matrix, matrix_to_quaternion

class StateBatch:
    """
    Stores the state of a given vehicle.
    
    Note:
        - position - A torch tensor with the [x,y,z] of the vehicle expressed in the inertial frame according to an ENU convention.
        - orientation - A torch tensor with the quaternion [qw, qx, qy, qz] that encodes the attitude of the vehicle's FLU body frame, relative to an ENU inertial frame, expressed in the ENU inertial frame.
        - linear_velocity - A torch tensor with [vx,vy,vz] that defines the velocity of the vehicle expressed in the inertial frame according to an ENU convention.
        - linear_body_velocity - A torch tensor with [u,v,w] that defines the velocity of the vehicle expressed in the FLU body frame.
        - angular_velocity - A torch tensor with [p,q,r] with the angular velocity of the vehicle's FLU body frame, relative to an ENU inertial frame, expressed in the FLU body frame.
        - linear acceleration - An array with [x_ddot, y_ddot, z_ddot] with the acceleration of the vehicle expressed in the inertial frame according to an ENU convention.
    """

    def __init__(self, n_vehicles, device):
        """
        Initialize the State object
        """

        # Define the same device that is running the simulation
        self.device = device

        # The position [x,y,z] of the vehicle's body frame relative to the inertial frame, expressed in the inertial frame
        self.position = torch.zeros((n_vehicles, 3), dtype=torch.float32, device=self.device)

        # The attitude (orientation) of the vehicle's body frame relative to the inertial frame of reference,
        # expressed in the inertial frame. This quaternion should follow the convention [qw, qx, qy, qz], such that "no rotation"
        # equates to the quaternion=[1, 0, 0, 0]
        self.attitude = torch.zeros((n_vehicles, 4), dtype=torch.float32, device=self.device)
        self.attitude[:, 0] = 1.0

        # The linear velocity [u,v,w] of the vehicle's body frame expressed in the body frame of reference
        self.linear_body_velocity = torch.zeros((n_vehicles, 3), dtype=torch.float32, device=self.device)

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        self.linear_velocity = torch.zeros((n_vehicles, 3), dtype=torch.float32, device=self.device)

        # The angular velocity [wx, wy, wz] of the vehicle's body frame relative to the inertial frame, expressed in the body frame
        self.angular_velocity = torch.zeros((n_vehicles, 3), dtype=torch.float32, device=self.device)

        # The linear acceleration [ax, ay, az] of the vehicle's body frame relative to the inertial frame, expressed in the inertial frame
        self.linear_acceleration = torch.zeros((n_vehicles, 3), dtype=torch.float32, device=self.device)


    def get_position_ned(self):
        """
        Method that, assuming that a state is encoded in ENU standard (the Isaac Sim standard), converts the position
        to the NED convention used by PX4 and other onboard flight controllers

        Returns:
            np.ndarray: A torch tensor with the [x,y,z] of the vehicle expressed in the inertial frame according to an NED convention.
        """
        return (rot_ENU_to_NED(device=self.device, dtype=torch.float32) @ self.position.T).T    # (n_vehicles, 3)


    def get_attitude_ned_frd(self):
        """
        Method that, assuming that a state is encoded in ENU-FLU standard (the Isaac Sim standard), converts the
        attitude of the vehicle it to the NED-FRD convention used by PX4 and other onboard flight controllers

        R_ENU_to_NED @ R_att @ R_FLU_to_FRD

        Returns:
            np.ndarray: A torch tensor with the quaternion [qw, qx, qy, qz] that encodes the attitude of the vehicle's FRD body frame, relative to an NED inertial frame, expressed in the NED inertial frame.
        """
        R_ENU_to_NED = rot_ENU_to_NED(device=self.device, dtype=self.attitude.dtype)      # (3,3)
        R_FLU_to_FRD = rot_FLU_to_FRD(device=self.device, dtype=self.attitude.dtype)      # (3,3)
        R_att = quaternion_to_matrix(self.attitude)                                       # (n_vehicles,3,3)

        attitude_frd_ned = R_ENU_to_NED.unsqueeze(0) @ R_att @ R_FLU_to_FRD.unsqueeze(0)    # (n_vehicles,3,3)

        return matrix_to_quaternion(attitude_frd_ned)       # (n_vehicles,4)


    def get_linear_body_velocity_ned_frd(self):
        """
        Method that, assuming that a state is encoded in ENU-FLU standard (the Isaac Sim standard), converts the
        linear body velocity of the vehicle it to the NED-FRD convention used by PX4 and other onboard flight controllers

        Returns:
            torch.Tensor: A torch tensor with (n_vehicles, 3) shape with [u, v, w] that defines the velocity of the vehicle expressed in the FRD body frame.
        """

        # Get the linear acceleration in FLU convention
        R_body_flu = quaternion_to_matrix(self.attitude)     # (n_vehicles, 3, 3)

        linear_acc_body_flu = torch.bmm(R_body_flu.transpose(1,2), self.linear_acceleration.unsqueeze(-1)).squeeze(-1)  # (n_vehicles,3)

        # Convert the linear acceleration in the body frame expressed in FLU convention to the FRD convention
        return (rot_FLU_to_FRD(device=self.device, dtype=torch.float32) @ linear_acc_body_flu.T).T    # (n_vehicles, 3)


    def get_linear_velocity_ned(self):
        """
        Method that, assuming that a state is enconded in ENU-FLU standard (the Isaac Sim standard), converts the
        linear velocity expressed in the inertial frame to the NED convention used by PX4 and other onboard flight
        controllers

        Returns:
            torch.Tensor: A torch tensor with (n_vehicles, 3) shape with [vx, vy, vz] that defines the velocity of the vehicle expressed in the inertial frame according to a NED convention.
        """
        return (rot_ENU_to_NED(device=self.device, dtype=torch.float32) @ self.linear_velocity.T).T    # (n_vehicles, 3)
        

    def get_angular_velocity_frd(self):
        """
        Method that, assuming that a state is enconded in ENU-FLU standard (the Isaac Sim standard), converts the
        angular velocity expressed in the body frame to the NED-FRD convention used by PX4 and other onboard flight
        controllers

        Returns:
            torch.Tensor: A torch tensor with (n_vehicles, 3) shape with [p, q, r] that defines the angular velocity of the vehicle's FRD body frame, relative to an NED inertial frame, expressed in the FRD body frame.
        """
        return (rot_FLU_to_FRD(device=self.device, dtype=torch.float32) @ self.angular_velocity.T).T    # (n_vehicles, 3)


    def get_linear_acceleration_ned(self):
        """
        Method that, assuming that a state is enconded in ENU-FLU standard (the Isaac Sim standard), converts the
        linear acceleration expressed in the inertial frame to the NED convention used by PX4 and other onboard flight
        controllers

        Returns:
            torch.Tensor: A torch tensor with (n_vehicles, 3) shape with [x_ddot, y_ddot, z_ddot] that defines the acceleration of the vehicle expressed in the inertial frame according to an NED convention.
        """
        return (rot_ENU_to_NED(device=self.device, dtype=torch.float32) @ self.linear_acceleration.T).T    # (n_vehicles, 3)

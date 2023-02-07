"""
| File: state.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Describes the state of a vehicle (or rigidbody).
"""
__all__ = ["State"]

import numpy as np
from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.rotations import rot_ENU_to_NED, rot_FLU_to_FRD


class State:
    """
    Stores the state of a given vehicle.
    
    Note:
        - position - A numpy array with the [x,y,z] of the vehicle expressed in the inertial frame according to an ENU convention.
        - orientation - A numpy array with the quaternion [qx, qy, qz, qw] that encodes the attitude of the vehicle's FLU body frame, relative to an ENU inertial frame, expressed in the ENU inertial frame.
        - linear_velocity - A numpy array with [vx,vy,vz] that defines the velocity of the vehicle expressed in the inertial frame according to an ENU convention.
        - linear_body_velocity - A numpy array with [u,v,w] that defines the velocity of the vehicle expressed in the FLU body frame.
        - angular_velocity - A numpy array with [p,q,r] with the angular velocity of the vehicle's FLU body frame, relative to an ENU inertial frame, expressed in the FLU body frame.
        - linear acceleration - An array with [x_ddot, y_ddot, z_ddot] with the acceleration of the vehicle expressed in the inertial frame according to an ENU convention.
    """

    def __init__(self):
        """
        Initialize the State object
        """

        # The position [x,y,z] of the vehicle's body frame relative to the inertial frame, expressed in the inertial frame
        self.position = np.array([0.0, 0.0, 0.0])

        # The attitude (orientation) of the vehicle's body frame relative to the inertial frame of reference,
        # expressed in the inertial frame. This quaternion should follow the convention [qx, qy, qz, qw], such that "no rotation"
        # equates to the quaternion=[0, 0, 0, 1]
        self.attitude = np.array([0.0, 0.0, 0.0, 1.0])

        # The linear velocity [u,v,w] of the vehicle's body frame expressed in the body frame of reference
        self.linear_body_velocity = np.array([0.0, 0.0, 0.0])

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        self.linear_velocity = np.array([0.0, 0.0, 0.0])

        # The angular velocity [wx, wy, wz] of the vehicle's body frame relative to the inertial frame, expressed in the body frame
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        # The linear acceleration [ax, ay, az] of the vehicle's body frame relative to the inertial frame, expressed in the inertial frame
        self.linear_acceleration = np.array([0.0, 0.0, 0.0])

    def get_position_ned(self):
        """
        Method that, assuming that a state is encoded in ENU standard (the Isaac Sim standard), converts the position
        to the NED convention used by PX4 and other onboard flight controllers

        Returns:
            np.ndarray: A numpy array with the [x,y,z] of the vehicle expressed in the inertial frame according to an NED convention.
        """
        return rot_ENU_to_NED.apply(self.position)

    def get_attitude_ned_frd(self):
        """
        Method that, assuming that a state is encoded in ENU-FLU standard (the Isaac Sim standard), converts the
        attitude of the vehicle it to the NED-FRD convention used by PX4 and other onboard flight controllers

        Returns:
            np.ndarray: A numpy array with the quaternion [qx, qy, qz, qw] that encodes the attitude of the vehicle's FRD body frame, relative to an NED inertial frame, expressed in the NED inertial frame.
        """
        attitude_frd_ned = rot_ENU_to_NED * Rotation.from_quat(self.attitude) * rot_FLU_to_FRD
        return attitude_frd_ned.as_quat()

    def get_linear_body_velocity_ned_frd(self):
        """
        Method that, assuming that a state is encoded in ENU-FLU standard (the Isaac Sim standard), converts the
        linear body velocity of the vehicle it to the NED-FRD convention used by PX4 and other onboard flight controllers

        Returns:
            np.ndarray: A numpy array with [u,v,w] that defines the velocity of the vehicle expressed in the FRD body frame.
        """

        # Get the linear acceleration in FLU convention
        linear_acc_body_flu = Rotation.from_quat(self.attitude).inv().apply(self.linear_acceleration)

        # Convert the linear acceleration in the body frame expressed in FLU convention to the FRD convention
        return rot_FLU_to_FRD.apply(linear_acc_body_flu)

    def get_linear_velocity_ned(self):
        """
        Method that, assuming that a state is enconded in ENU-FLU standard (the Isaac Sim standard), converts the
        linear velocity expressed in the inertial frame to the NED convention used by PX4 and other onboard flight
        controllers

        Returns:
            np.ndarray: A numpy array with [vx,vy,vz] that defines the velocity of the vehicle expressed in the inertial frame according to a NED convention.
        """
        return rot_ENU_to_NED.apply(self.linear_velocity)

    def get_angular_velocity_frd(self):
        """
        Method that, assuming that a state is enconded in ENU-FLU standard (the Isaac Sim standard), converts the
        angular velocity expressed in the body frame to the NED-FRD convention used by PX4 and other onboard flight
        controllers

        Returns:
            np.ndarray: A numpy array with [p,q,r] with the angular velocity of the vehicle's FRD body frame, relative to an NED inertial frame, expressed in the FRD body frame.
        """
        return rot_FLU_to_FRD.apply(self.angular_velocity)

    def get_linear_acceleration_ned(self):
        """
        Method that, assuming that a state is enconded in ENU-FLU standard (the Isaac Sim standard), converts the
        linear acceleration expressed in the inertial frame to the NED convention used by PX4 and other onboard flight
        controllers

        Returns:
            np.ndarray: An array with [x_ddot, y_ddot, z_ddot] with the acceleration of the vehicle expressed in the inertial frame according to an NED convention.
        """
        return rot_ENU_to_NED.apply(self.linear_acceleration)

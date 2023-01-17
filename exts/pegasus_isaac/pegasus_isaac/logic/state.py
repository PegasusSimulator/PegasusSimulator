#!/usr/bin/env python

__all__ = ["State"]

import numpy as np
from scipy.spatial.transform import Rotation
from pegasus_isaac.logic.rotations import rot_ENU_to_NED, rot_FLU_to_FRD

class State:
    """
    Stores the state of a given vehicle, as perceived by Isaac Sim
    """

    def __init__(self):
        
        # The position [x,y,z] of the vehicle's body frame relative to the inertial frame, expressed in the inertial frame
        self.position = np.array([0.0, 0.0, 0.0])

        # The attitude (orientation) of the vehicle's body frame relative to the inertial frame of reference, 
        # expressed in the inertial frame. This quaternion should follow the convention [qx, qy, qz, qw], such that "no rotation"
        # equates to the quaternion=[0, 0, 0, 1]
        self.attitude = np.array([0.0, 0.0, 0.0, 1.0])

        # The linear velocity [u,v,w] of the vehicle's body frame expressed in the body frame of reference
        self.linear_body_velocity = np.array([0.0, 0.0, 0.])

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
        """
        return rot_ENU_to_NED.apply(self.position)

    def get_orientation_ned_frd(self):
        """
        Method that, assuming that a state is encoded in ENU-FLU standard (the Isaac Sim standard), converts the 
        attitude of the vehicle it to the NED-FRD convention used by PX4 and other onboard flight controllers
        """
        attitude_frd_ned = rot_ENU_to_NED * Rotation.from_quat(self.attitude) * rot_FLU_to_FRD
        return attitude_frd_ned.as_quat()

    def get_linear_body_velocity_ned_frd(self):
        """
        Method that, assuming that a state is encoded in ENU-FLU standard (the Isaac Sim standard), converts the 
        linear body velocity of the vehicle it to the NED-FRD convention used by PX4 and other onboard flight controllers
        """
        pass

    def get_linear_velocity_ned(self):
        """
        Method that, assuming that a state is enconded in ENU-FLU standard (the Isaac Sim standard), converts the 
        linear velocity expressed in the inertial frame to the NED convention used by PX4 and other onboard flight
        controllers
        """
        return rot_ENU_to_NED.apply(self.linear_velocity)

    def get_linear_acceleration_ned(self):
        """
        Method that, assuming that a state is enconded in ENU-FLU standard (the Isaac Sim standard), converts the 
        linear acceleration expressed in the inertial frame to the NED convention used by PX4 and other onboard flight
        controllers
        """
        return rot_ENU_to_NED.apply(self.linear_acceleration)

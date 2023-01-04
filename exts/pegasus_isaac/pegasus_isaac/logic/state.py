#!/usr/bin/env python

import numpy as np

class State:
    """
    Stores the state of a given vehicle, as perceived by Isaac Sim
    """

    def __init__(self):
        
        # The position [x,y,z] of the vehicle's body frame relative to the inertial frame, expressed in the inertial frame
        self.position = np.array([0.0, 0.0, 0.0])

        # The attitude (orientation) of the vehicle's body frame relative to the inertial frame of reference, 
        # expressed in the inertial frame
        self.attitude = np.array([0.0, 0.0, 0.0, 1.0])

        # The linear velocity [u,v,w] of the vehicle's body frame expressed in the body frame of reference
        self.linear_body_velocity = np.array([0.0, 0.0, 0.])

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        self.linear_velocity = np.array([0.0, 0.0, 0.0])
        
        # The angular velocity [wx, wy, wz] of the vehicle's body frame relative to the inertial frame, expressed in the body frame
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        # The linear acceleration [ax, ay, az] of the vehicle's body frame relative to the inertial frame, expressed in the inertial frame
        self.linear_acceleration = np.array([0.0, 0.0, 0.0])

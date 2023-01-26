#!/usr/bin/env python
"""
File: linear_drag.py
Author: Marcelo Jacinto
Email: marcelo.jacinto@tecnico.ulisboa.pt
Github: https://github.com/marcelojacinto
Description:
    Computes the forces that should actuate on a rigidbody affected by linear drag
"""
import numpy as np
from pegasus_isaac.logic.state import State

class LinearDrag:

    def __init__(self, drag_coefficients=[0.0, 0.0, 0.0]):
        """
        Receives as input the drag coefficients of the vehicle as a 3x1 vector of constants
        """

        # The linear drag coefficients of the vehicle's body frame
        self._drag_coefficients = np.diag(drag_coefficients)

        # The drag force to apply on the vehicle's body frame
        self._drag_force = np.array([0.0, 0.0, 0.0])

    @property
    def drag(self):
        return self._drag_force

    def update(self, state: State, dt: float):
        
        # Get the velocity of the vehicle expressed in the body frame of reference
        body_vel = state.linear_body_velocity

        # Compute the component of the drag force to be applied in the body frame
        self._drag_force = np.dot(self._drag_coefficients, body_vel)
        return self._drag_force

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

    def __init__(self, drag_coefficients: np.ndarray):
        """
        Receives as input the drag coefficients of the vehicle 
        """

        self._drag = np.diag(drag_coefficients)

    @property
    def drag(self):
        return self._drag

    def update(self, state: State, dt: float):
        
        # Get the velocity of the vehicle expressed in the body frame of reference
        body_vel = state.linear_body_velocity

        # Compute the component of the drag force to be applied in the body frame
        return self.drag * body_vel

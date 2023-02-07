"""
| File: drag.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: Base interface used to implement forces that should actuate on a rigidbody such as linear drag
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""
from pegasus.simulator.logic.state import State

class Drag:
    """
    Class that serves as a template for the implementation of Drag forces that actuate on a rigid body
    """

    def __init__(self):
        """
        Receives as input the drag coefficients of the vehicle as a 3x1 vector of constants
        """

    @property
    def drag(self):
        """The drag force to be applied on the body frame of the vehicle

        Returns:
            list: A list with len==3 containing the drag force to be applied on the rigid body according to a FLU body reference
            frame, expressed in Newton (N) [dx, dy, dz]
        """
        return [0.0, 0.0, 0.0]

    def update(self, state: State, dt: float):
        """Method that should be implemented to update the drag force to be applied on the body frame of the vehicle

        Args:
            state (State): The current state of the vehicle.
             dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            list: A list with len==3 containing the drag force to be applied on the rigid body according to a FLU body reference
        """
        return [0.0, 0.0, 0.0]

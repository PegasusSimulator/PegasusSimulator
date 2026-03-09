"""
| File: linear_drag.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: Computes the forces that should actuate on a rigidbody affected by linear drag
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""
import torch
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.dynamics.drag import Drag
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface


class LinearDragBatch(Drag):
    """
    Class that implements linear drag computations afftecting a rigid body. It inherits the Drag base class.
    """

    def __init__(self, n_vehicles=1, drag_coefficients=[0.0, 0.0, 0.0]):
        """
        Receives as input the drag coefficients of the vehicle as a 3x1 vector of constants

        Args:
            drag_coefficients (list[float]): The constant linear drag coefficients to used to compute the total drag forces
            affecting the rigid body. The linear drag is given by diag(dx, dy, dz) * [v_x, v_y, v_z] where the velocities
            are expressed in the body frame of the rigid body (using the FRU frame convention).
        """

        # Initialize the base Drag class
        super().__init__()

        # Define the same device that is running the simulation
        self.device = PegasusInterface()._world_settings["device"]

        # The linear drag coefficients of the vehicle's body frame
        self._drag_coefficients = torch.tensor(drag_coefficients, dtype=torch.float32, device=self.device)        
        
        # The drag force to apply on the vehicle's body frame
        self._drag_force = torch.zeros((n_vehicles, 3), dtype=torch.float32, device=self.device)


    @property
    def drag(self):
        """The drag force to be applied on the body frame of the vehicle

        Returns:
            list: A list with len==3 containing the drag force to be applied on the rigid body according to a FLU body reference
            frame, expressed in Newton (N) [dx, dy, dz]
        """
        return self._drag_force


    def update(self, state: State, dt: float):
        """Method that updates the drag force to be applied on the body frame of the vehicle. The total drag force
        applied on the body reference frame (FLU convention) is given by diag(dx,dy,dz) * R' * v
        where v is the velocity of the vehicle expressed in the inertial frame and R' * v = velocity_body_frame

        Args:
            state (State): The current state of the vehicle.
             dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            list: A list with len==3 containing the drag force to be applied on the rigid body according to a FLU body reference
        """

        # Get the velocity of the vehicle expressed in the body frame of reference
        body_vel = state.linear_body_velocity       # (n_vehicles, 3)

        self._drag_force = -(body_vel * self._drag_coefficients)

        # Compute the component of the drag force to be applied in the body frame
        return self._drag_force

"""
| File: line_person_controller.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: Controller class that can be used to make a person follow a line in the simulation
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
"""
import numpy as np
from pegasus.simulator.logic.people.person_controller import PersonController

class LinePersonController(PersonController):

    def __init__(self, start, stop, speed=0.1):
        """Line Person Controller that makes a person follow a line in the simulation

        Args:
            start (np.array): The starting point of the line
            stop (np.array): The ending point of the line
            speed (float, optional): The speed in m/s at which to follow the line. Defaults to 0.1.
        """

        super().__init__()

        self.slope = stop - start
        self.gamma = 0.0

        # Compute the desired speed in the parameterized units (0 - 1)
        derivative_norm = np.linalg.norm(self.slope)
        if derivative_norm == 0:
            self.gamma_dot = 0
        else:
            self.gamma_dot = speed / derivative_norm
        
    def update(self, dt: float):

        # Get the desired position of the person
        desired_position = self.slope * self.gamma

        # Update the reference position for the person to track
        self.gamma += self.gamma_dot * dt

        # Ensure that gamma is between 0 and 1
        if self.gamma > 1.0:
            self.gamma

        # Set the target position for the person to track
        self._person.update_target_position(desired_position)
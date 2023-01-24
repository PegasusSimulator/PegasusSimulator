#!/usr/bin/env python
"""
File: quadratic_thrust_curve.py
Author: Marcelo Jacinto
Email: marcelo.jacinto@tecnico.ulisboa.pt
Github: https://github.com/marcelojacinto
Description:
    Computes the forces that should actuate on each rotor given a desired angular velocity
"""
import numpy as np
from pegasus_isaac.logic.state import State
class QuadraticThrustCurve:

    def __init__(self, 
        num_rotors: int=4, 
        rotor_constant=[5.84E-6, 5.84E-6, 5.84E-6, 5.84E-6],
        rolling_moment_coefficient=[1E-6, 1E-6, 1E-6, 1E-6],
        min_rotor_velocity=[0, 0, 0, 0],
        max_rotor_velocity=[1100, 1100, 1100, 1100],
        # Rotation direction for the rotors
        rot_dir=[-1,-1,1,1]):
        
        # Get the total number of rotors to simulate
        self._num_rotors = num_rotors

        # The rotor constant used for computing the total thrust produced by the rotor: T = rotor_constant * omega^2
        assert len(rotor_constant) == self._num_rotors
        self._rotor_constant = rotor_constant

        # The rotor constant used for computing the total torque generated about the vehicle Z-axis
        assert len(rolling_moment_coefficient) == self._num_rotors
        self._rolling_moment_coefficient = rolling_moment_coefficient

        # Save the rotor direction of rotation
        assert len(rot_dir) == self._num_rotors
        self._rot_dir = rot_dir

        # Values for the minimum and maximum rotor velocity in rad/s
        assert len(min_rotor_velocity) == self._num_rotors
        self.min_rotor_velocity = min_rotor_velocity

        assert len(max_rotor_velocity) == self._num_rotors
        self.max_rotor_velocity = max_rotor_velocity
        
        # The actual speed references to apply to the vehicle rotor joints
        self._input_reference = [0.0 for i in range(self._num_rotors)]

        # The actual velocity that each rotor is spinning at
        self._velocity = [0.0 for i in range(self._num_rotors)]

        # The actual force that each rotor is generating
        self._force = [0.0 for i in range(self._num_rotors)]

        # The actual rolling moment that is generated on the body frame of the vehicle
        self._rolling_moment = 0.0

    def set_input_reference(self, input_reference):
        """
        Receives as input a list of target angular velocities of each rotor in rad/s
        """

        # The target angular velocity of the rotor
        self._input_reference = input_reference

    def update(self, state: State, dt: float):
        """
        Note: the state and dt variables are not used in this implementation, but left
        to add support to other rotor models where the total thrust is dependent on 
        states such as vehicle linear velocity
        """

        rolling_moment = 0.0
        
        # Compute the actual force to apply to the rotors and the rolling moment contribution
        for i in range(self._num_rotors):

            # Set the actual velocity that each rotor is spinning at (instanenous model - no delay introduced)
            # Only apply clipping of the input reference
            self._velocity[i] = np.maximum(self.min_rotor_velocity[i], np.minimum(self._input_reference[i], self.max_rotor_velocity[i]))

            # Set the force using a quadratic thrust curve
            self._force[i] = self._rotor_constant[i] * np.power(self._velocity[i], 2)

            # Compute the rolling moment coefficient
            rolling_moment += self._rolling_moment_coefficient[i] * np.power(self._velocity[i], 2.0) * self._rot_dir[i]

        # Update the rolling moment variable
        self._rolling_moment = rolling_moment

        # Return the forces and velocities on each rotor and total torque applied on the body frame
        return self._force, self._velocity, self._rolling_moment

    @property
    def force(self):
        return self._force

    @property
    def velocity(self):
        return self._velocity

    @property
    def rolling_moment(self):
        return self._rolling_moment
        
    @property
    def rot_dir(self):
        return self._rot_dir

"""
| File: thrust_curve.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Descriptio: File that implements the base interface for defining thrust curves for vehicles
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""
from pegasus.simulator.logic.state import State


class ThrustCurve:
    """Class that implements the dynamics of rotors that can be described by a quadratic thrust curve
    """
    def __init__(self):
        pass

    def set_input_reference(self, input_reference):
        """
        Receives as input a list of target angular velocities of each rotor in rad/s
        """
        pass

    def update(self, state: State, dt: float):
        """
        Note: the state and dt variables are not used in this implementation, but left
        to add support to other rotor models where the total thrust is dependent on
        states such as vehicle linear velocity

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        pass

    @property
    def force(self):
        """The force to apply to each rotor of the vehicle at any given time instant

        Returns:
            list: A list of forces (in Newton N) to apply to each rotor of the vehicle (on its Z-axis) at any given time instant
        """
        pass

    @property
    def velocity(self):
        """The velocity at which each rotor of the vehicle should be rotating at any given time instant

        Returns:
            list: A list of angular velocities (in rad/s) of each rotor (about its Z-axis) at any given time instant
        """
        pass

    @property
    def rolling_moment(self):
        """The total rolling moment being generated on the body frame of the vehicle by the rotating propellers

        Returns:
            float: The total rolling moment to apply to the vehicle body frame (Torque about the Z-axis) in Nm
        """
        pass

    @property
    def rot_dir(self):
        """The direction of rotation of each rotor of the vehicle

        Returns:
            list(int): A list with the rotation direction of each rotor (-1 is counter-clockwise and 1 for clockwise)
        """
        pass
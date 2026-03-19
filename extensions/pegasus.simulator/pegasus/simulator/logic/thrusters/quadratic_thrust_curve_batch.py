"""
| File: quadratic_thrust_curve.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Descriptio: File that implements a quadratic thrust curve for rotors
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""
import torch
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.thrusters.thrust_curve import ThrustCurve

class QuadraticThrustCurveBatch(ThrustCurve):
    """Class that implements the dynamics of rotors that can be described by a quadratic thrust curve
    """
    def __init__(self, config={}, n_vehicles = 1, device="cpu"):
        """
        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the QuadraticThrustCurve - it can be empty or only have some of the parameters used by the QuadraticThrustCurve.
            n_vehicles (int): The number of vehicles for which to simulate the thrust curve.
            device (str): The device on which to run the simulation.

        Examples:
            The dictionary default parameters are

            >>> {"num_rotors": 4,
            >>>  "rotor_constant": [5.84e-6, 5.84e-6, 5.84e-6, 5.84e-6],
            >>>  "rolling_moment_coefficient": [1e-6, 1e-6, 1e-6, 1e-6],
            >>>  "rot_dir": [-1, -1, 1, 1],
            >>>  "min_rotor_velocity": [0, 0, 0, 0],                      # rad/s
            >>>  "max_rotor_velocity": [1100, 1100, 1100, 1100],          # rad/s
            >>> }
        """
        # Set device
        self.device = device

        # Get the total number of rotors to simulate
        self._num_rotors = config.get("num_rotors", 4)

        # Set the number of vehicles to construct batches of forces and velocities for each vehicle 
        self.n_vehicles = n_vehicles

        # The rotor constant used for computing the total thrust produced by the rotor: T = rotor_constant * omega^2
        self._rotor_constant = torch.tensor(config.get("rotor_constant", [8.54858e-6, 8.54858e-6, 8.54858e-6, 8.54858e-6]), dtype=torch.float32, device=self.device)
        assert len(self._rotor_constant) == self._num_rotors

        # The rotor constant used for computing the total torque generated about the vehicle Z-axis
        self._rolling_moment_coefficient = torch.tensor(config.get("rolling_moment_coefficient", [1e-6, 1e-6, 1e-6, 1e-6]), dtype=torch.float32, device=self.device)
        assert len(self._rolling_moment_coefficient) == self._num_rotors

        # Save the rotor direction of rotation
        self._rot_dir = torch.tensor(config.get("rot_dir", [-1, -1, 1, 1]), dtype=torch.int32, device=self.device)
        assert len(self._rot_dir) == self._num_rotors

        # Values for the minimum and maximum rotor velocity in rad/s
        self.min_rotor_velocity = torch.tensor(config.get("min_rotor_velocity", [0, 0, 0, 0]), dtype=torch.float32, device=self.device)
        assert len(self.min_rotor_velocity) == self._num_rotors

        self.max_rotor_velocity = torch.tensor(config.get("max_rotor_velocity", [1100, 1100, 1100, 1100]), dtype=torch.float32, device=self.device)
        assert len(self.max_rotor_velocity) == self._num_rotors

        # The actual speed references to apply to the vehicle rotor joints
        self._input_reference = torch.zeros((self.n_vehicles, self._num_rotors), dtype=torch.float32, device=self.device)
        
        # The actual velocity that each rotor is spinning at
        self._velocity = torch.zeros((self.n_vehicles, self._num_rotors), dtype=torch.float32, device=self.device)

        # The actual force that each rotor is generating
        self._force = torch.zeros((self.n_vehicles, self._num_rotors), dtype=torch.float32, device=self.device)

        # The actual rolling moment that is generated on the body frame of the vehicle
        self._rolling_moment = torch.zeros(self.n_vehicles, dtype=torch.float32, device=self.device)

    def set_input_reference(self, input_reference):
        """
        Receives as input a list of target angular velocities of each rotor in rad/s

        Args:
            input_reference (torch.Tensor): A tensor of shape (n_veiculos, num_rotors)
        """

        if input_reference.ndim == 1:
            if input_reference.shape[0] != self._num_rotors:
                raise ValueError(
                    f"input_reference must have {self._num_rotors} rotors, got {input_reference.shape[0]}"
                )
            input_reference = input_reference.unsqueeze(0).expand(self.n_vehicles, -1)

        elif input_reference.ndim == 2:
            if input_reference.shape != (self.n_vehicles, self._num_rotors):
                raise ValueError(
                    f"input_reference must have shape "
                    f"({self.n_vehicles}, {self._num_rotors}), got {tuple(input_reference.shape)}"
                )
        else:
            raise ValueError("input_reference must be 1D or 2D")

        # The target angular velocity of the rotor
        self._input_reference = torch.as_tensor(input_reference, dtype=torch.float32, device=self.device)

    def update(self, state: State, dt: float):
        """
        Note: the state and dt variables are not used in this implementation, but left
        to add support to other rotor models where the total thrust is dependent on
        states such as vehicle linear velocity

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Broadcast min/max from (num_rotors,) to (n_vehicles, num_rotors)
        min_vel = self.min_rotor_velocity.unsqueeze(0)
        max_vel = self.max_rotor_velocity.unsqueeze(0)

        # Set the actual velocity that each rotor is spinning at (instanenous model - no delay introduced)
        # Only apply clipping of the input reference
        self._velocity = torch.clamp(self._input_reference, min=min_vel, max=max_vel)

        # Set the force using a quadratic thrust curve
        self._force = self._rotor_constant.unsqueeze(0) * torch.pow(self._velocity, 2)

        # Compute the rolling moment coefficient
        self._rolling_moment = torch.sum(self._rolling_moment_coefficient.unsqueeze(0) * torch.pow(self._velocity, 2) * self._rot_dir.unsqueeze(0), dim=1)

        # Return the forces and velocities on each rotor and total torque applied on the body frame
        return self._force, self._velocity, self._rolling_moment


    @property
    def force(self):
        """The force to apply to each rotor of the vehicle at any given time instant

        Returns:
            list: A list of forces (in Newton N) to apply to each rotor of the vehicle (on its Z-axis) at any given time instant
        """
        return self._force

    @property
    def velocity(self):
        """The velocity at which each rotor of the vehicle should be rotating at any given time instant

        Returns:
            list: A list of angular velocities (in rad/s) of each rotor (about its Z-axis) at any given time instant
        """
        return self._velocity

    @property
    def rolling_moment(self):
        """The total rolling moment being generated on the body frame of the vehicle by the rotating propellers

        Returns:
            float: The total rolling moment to apply to the vehicle body frame (Torque about the Z-axis) in Nm
        """
        return self._rolling_moment

    @property
    def rot_dir(self):
        """The direction of rotation of each rotor of the vehicle

        Returns:
            list(int): A list with the rotation direction of each rotor (-1 is counter-clockwise and 1 for clockwise)
        """
        return self._rot_dir

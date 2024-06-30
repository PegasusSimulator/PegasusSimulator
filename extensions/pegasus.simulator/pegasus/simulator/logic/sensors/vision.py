"""
| File: vision.py
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto and Filip Stec. All rights reserved.
| Description: Simulates a visual odometry. Based on the implementation provided in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo) by Amy Wagoner and Nuno Marques
"""
__all__ = ["Vision"]

import numpy as np
from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.sensors import Sensor

class Vision(Sensor):
    """The class that implements a Vision sensor. This class inherits the base class Sensor.
    """

    def __init__(self, config={}):
        """Initialize the Vision class.

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the Vision - it can be empty or only have some of the parameters used by the Vision.
        
        Examples:
            The dictionary default parameters are

            >>> {"reset_counter": 0,
            >>>  "vision_random_walk": 0.1,         # (m/s) / sqrt(hz)
            >>>  "vision_noise_density": 0.01,      # (m) / sqrt(hz)
            >>>  "vision_correlation_time": 60,     # s
            >>>  "update_rate": 30.0                # Hz
            >>> }
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="Vision", update_rate=config.get("update_rate", 30.0))

        # Define the Vision simulated/fixed values
        self._reset_counter = config.get("reset_counter", 0)

        # Parameters for Vision random walk
        self._random_walk = np.array([0.0, 0.0, 0.0])
        self._vision_random_walk = config.get("vision_random_walk", 0.1)

        # Parameters for Vision position and linear/angular velocity noise
        self._noise_pos = np.array([0.0, 0.0, 0.0])
        self._noise_linvel = np.array([0.0, 0.0, 0.0])
        self._noise_angvel = np.array([0.0, 0.0, 0.0])
        self._vision_noise_density = config.get("vision_noise_density", 0.01)

        # Parameters for Vision bias
        self._bias = np.array([0.0, 0.0, 0.0])
        self._vision_correlation_time = config.get("vision_correlation_time", 60.0)

        # Position covariance is constant, so prepare it in advance
        self._vision_covariance = np.array(
            [self._vision_noise_density * self._vision_noise_density if i in [0, 6, 11, 15, 18, 20] else 0.0 for i in range(21)], 
            dtype=float)

        # Save the current state measured by the GPS (and initialize at the origin)
        self._state = {
            "x": 0.0,
            "y": 0.0, 
            "z": 0.0,
            "roll": 0.0, 
            "pitch": 0.0, 
            "yaw": 0.0,
            "covariance": self._vision_covariance,
            "reset_counter": self._reset_counter,
        }
    
    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state
    
    @Sensor.update_at_rate
    def update(self, state: np.ndarray, dt: float):
        """Method that implements the logic of a visual odometry.

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        # Update noise parameters
        self._random_walk[0] = self._vision_random_walk * np.sqrt(dt) * np.random.randn()
        self._random_walk[1] = self._vision_random_walk * np.sqrt(dt) * np.random.randn()
        self._random_walk[2] = self._vision_random_walk * np.sqrt(dt) * np.random.randn()

        self._noise_pos[0] = self._vision_noise_density * np.sqrt(dt) * np.random.randn()
        self._noise_pos[1] = self._vision_noise_density * np.sqrt(dt) * np.random.randn()
        self._noise_pos[2] = self._vision_noise_density * np.sqrt(dt) * np.random.randn()

        self._noise_linvel[0] = self._vision_noise_density * np.sqrt(dt) * np.random.randn()
        self._noise_linvel[1] = self._vision_noise_density * np.sqrt(dt) * np.random.randn()
        self._noise_linvel[2] = self._vision_noise_density * np.sqrt(dt) * np.random.randn()

        tau_g = self._vision_correlation_time
        sigma_g_d = 1 / np.sqrt(dt) * self._vision_noise_density
        sigma_b_g = self._vision_random_walk
        sigma_b_g_d = np.sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 * (np.exp(-2.0 * dt / tau_g) - 1.0))
        phi_g_d = np.exp(-1.0 / tau_g * dt)

        self._noise_angvel[0] = phi_g_d * self._noise_angvel[0] + sigma_b_g_d * np.sqrt(dt) * np.random.randn() # self._noise_angvel[0] might need to be 0.0
        self._noise_angvel[1] = phi_g_d * self._noise_angvel[1] + sigma_b_g_d * np.sqrt(dt) * np.random.randn()
        self._noise_angvel[2] = phi_g_d * self._noise_angvel[2] + sigma_b_g_d * np.sqrt(dt) * np.random.randn()

        # Perform Vision bias integration
        self._bias[0] = (
            self._bias[0] + self._random_walk[0] * dt - self._bias[0] / self._vision_correlation_time
        )
        self._bias[1] = (
            self._bias[1] + self._random_walk[1] * dt - self._bias[1] / self._vision_correlation_time
        )
        self._bias[2] = (
            self._bias[2] + self._random_walk[2] * dt - self._bias[2] / self._vision_correlation_time
        )

        # Get resulting values 
        position: np.ndarray = state.get_position_ned() + self._noise_pos + self._bias
        orientation: np.ndarray = Rotation.from_quat(state.get_attitude_ned_frd()).as_euler('xyz', degrees=False)
        linear_velocity: np.ndarray = state.get_linear_velocity_ned() + self._noise_linvel
        angular_velocity: np.ndarray = state.get_angular_velocity_frd() + self._noise_angvel

        self._state = {
            "x": position[0],
            "y": position[1],
            "z": position[2],
            "roll": orientation[0],
            "pitch": orientation[1],
            "yaw": orientation[2],
            "covariance": self._vision_covariance,
            "reset_counter": self._reset_counter,
        }

        return self._state

"""
| File: imu.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Simulates an imu. Based on the implementation provided in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo)
"""
__all__ = ["IMU"]

import numpy as np
from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.rotations import rot_FLU_to_FRD, rot_ENU_to_NED
from pegasus.simulator.logic.sensors.geo_mag_utils import GRAVITY_VECTOR


class IMU(Sensor):
    """The class that implements the IMU sensor. This class inherits the base class Sensor.
    """
    def __init__(self, config={}):
        """Initialize the IMU class

        Args:
            config (dict): A Dictionary that contains all teh parameters for configuring the IMU - it can be empty or only have some of the parameters used by the IMU.

        Examples:
            The dictionary default parameters are

            >>> {"gyroscope": {
            >>>        "noise_density": 2.0 * 35.0 / 3600.0 / 180.0 * pi,
            >>>        "random_walk": 2.0 * 4.0 / 3600.0 / 180.0 * pi,
            >>>        "bias_correlation_time": 1.0e3,
            >>>        "turn_on_bias_sigma": 0.5 / 180.0 * pi},
            >>>  "accelerometer": {
            >>>         "noise_density": 2.0 * 2.0e-3,
            >>>         "random_walk": 2.0 * 3.0e-3,
            >>>         "bias_correlation_time": 300.0,
            >>>         "turn_on_bias_sigma": 20.0e-3 * 9.8
            >>> },
            >>>  "update_rate": 1.0}                 # Hz
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="IMU", update_rate=config.get("update_rate", 250.0))

        # Orientation noise constant
        self._orientation_noise: float = 0.0

        # Gyroscope noise constants
        self._gyroscope_bias: np.ndarray = np.zeros((3,))
        gyroscope_config = config.get("gyroscope", {})
        self._gyroscope_noise_density = gyroscope_config.get("noise_density", 0.0003393695767766752)
        self._gyroscope_random_walk = gyroscope_config.get("random_walk", 3.878509448876288E-05)
        self._gyroscope_bias_correlation_time = gyroscope_config.get("bias_correlation_time", 1.0E3)
        self._gyroscope_turn_on_bias_sigma = gyroscope_config.get("turn_on_bias_sigma", 0.008726646259971648)

        # Accelerometer noise constants
        self._accelerometer_bias: np.ndarray = np.zeros((3,))
        accelerometer_config = config.get("accelerometer", {})
        self._accelerometer_noise_density = accelerometer_config.get("noise_density", 0.004)
        self._accelerometer_random_walk = accelerometer_config.get("random_walk", 0.006)
        self._accelerometer_bias_correlation_time = accelerometer_config.get("bias_correlation_time", 300.0)
        self._accelerometer_turn_on_bias_sigma = accelerometer_config.get("turn_on_bias_sigma", 0.196)

        # Auxiliar variable used to compute the linear acceleration of the vehicle
        self._prev_linear_velocity = np.zeros((3,))

        # Save the current state measured by the IMU
        self._state = {
            "orientation": np.array([1.0, 0.0, 0.0, 0.0]),
            "angular_velocity": np.array([0.0, 0.0, 0.0]),
            "linear_acceleration": np.array([0.0, 0.0, 0.0]),
        }

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state

    @Sensor.update_at_rate
    def update(self, state: State, dt: float):
        """Method that implements the logic of an IMU. In this method we start by generating the random walk of the 
        gyroscope. This value is then added to the real angular velocity of the vehicle (FLU relative to ENU inertial frame
        expressed in FLU body frame). The same logic is followed for the accelerometer and the accelerations. After this step,
        the angular velocity is rotated such that it expressed a FRD body frame, relative to a NED inertial frame, expressed
        in the FRD body frame. Additionally, the acceleration is also rotated, such that it becomes expressed in the body
        FRD frame of the vehicle. This sensor outputs data that follows the PX4 adopted standard.

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        # Gyroscopic terms
        tau_g: float = self._accelerometer_bias_correlation_time

        # Discrete-time standard deviation equivalent to an "integrating" sampler with integration time dt
        sigma_g_d: float = 1 / np.sqrt(dt) * self._gyroscope_noise_density
        sigma_b_g: float = self._gyroscope_random_walk

        # Compute exact covariance of the process after dt [Maybeck 4-114]
        sigma_b_g_d: float = np.sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 * (np.exp(-2.0 * dt / tau_g) - 1.0))

        # Compute state-transition
        phi_g_d: float = np.exp(-1.0 / tau_g * dt)

        # Simulate gyroscope noise processes and add them to the true angular rate.
        angular_velocity: np.ndarray = np.zeros((3,))

        for i in range(3):
            self._gyroscope_bias[i] = phi_g_d * self._gyroscope_bias[i] + sigma_b_g_d * np.random.randn()
            angular_velocity[i] = state.angular_velocity[i] + sigma_g_d * np.random.randn() + self._gyroscope_bias[i]

        # Accelerometer terms
        tau_a: float = self._accelerometer_bias_correlation_time

        # Discrete-time standard deviation equivalent to an "integrating" sampler with integration time dt
        sigma_a_d: float = 1.0 / np.sqrt(dt) * self._accelerometer_noise_density
        sigma_b_a: float = self._accelerometer_random_walk

        # Compute exact covariance of the process after dt [Maybeck 4-114].
        sigma_b_a_d: float = np.sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 * (np.exp(-2.0 * dt / tau_a) - 1.0))

        # Compute state-transition.
        phi_a_d: float = np.exp(-1.0 / tau_a * dt)

        # Compute the linear acceleration from diferentiating the velocity of the vehicle expressed in the inertial frame
        linear_acceleration_inertial = (state.linear_velocity - self._prev_linear_velocity) / dt
        linear_acceleration_inertial = linear_acceleration_inertial - GRAVITY_VECTOR

        # Update the previous linear velocity for the next computation
        self._prev_linear_velocity = state.linear_velocity

        # Compute the linear acceleration of the body frame, with respect to the inertial frame, expressed in the body frame
        linear_acceleration = np.array(Rotation.from_quat(state.attitude).inv().apply(linear_acceleration_inertial))

        # Simulate the accelerometer noise processes and add them to the true linear aceleration values
        for i in range(3):
            self._accelerometer_bias[i] = phi_a_d * self._accelerometer_bias[i] + sigma_b_a_d * np.random.rand()
            linear_acceleration[i] = (
                linear_acceleration[i] + sigma_a_d * np.random.randn()
            ) #+ self._accelerometer_bias[i]

        # TODO - Add small "noisy" to the attitude

        # --------------------------------------------------------------------------------------------
        # Apply rotations such that we express the IMU data according to the FRD body frame convention
        # --------------------------------------------------------------------------------------------

        # Convert the orientation to the FRD-NED standard
        attitude_flu_enu = Rotation.from_quat(state.attitude)
        attitude_frd_enu = attitude_flu_enu * rot_FLU_to_FRD
        attitude_frd_ned = rot_ENU_to_NED * attitude_frd_enu

        # Convert the angular velocity from FLU to FRD standard
        angular_velocity_frd = rot_FLU_to_FRD.apply(angular_velocity)

        # Convert the linear acceleration in the body frame from FLU to FRD standard
        linear_acceleration_frd = rot_FLU_to_FRD.apply(linear_acceleration)

        # Add the values to the dictionary and return it
        self._state = {
            "orientation": attitude_frd_ned.as_quat(),
            "angular_velocity": angular_velocity_frd,
            "linear_acceleration": linear_acceleration_frd,
        }

        return self._state

"""
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Simulates a magnetometer. Based on the original implementation provided in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo) by Elia Tarasov
"""
__all__ = ["Magnetometer"]

import numpy as np
from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.rotations import rot_ENU_to_NED, rot_FLU_to_FRD
from pegasus.simulator.logic.sensors.geo_mag_utils import (
    get_mag_declination,
    get_mag_inclination,
    get_mag_strength,
    reprojection,
)

class Magnetometer(Sensor):
    """The class that implements a magnetometer sensor. This class inherits the base class Sensor.
    """

    def __init__(self, config={}):
        """Initialize the Magnetometer class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the Magnetometer - it can be empty or only have some of the parameters used by the Magnetometer.
            
        Examples:
            The dictionary default parameters are

            >>> {"noise_density": 0.4e-3,           # gauss / sqrt(hz)
            >>>  "random_walk": 6.4e-6,             # gauss * sqrt(hz)
            >>>  "bias_correlation_time": 6.0e2,    # s
            >>>  "update_rate": 250.0}              # Hz
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="Magnetometer", update_rate=config.get("update_rate", 250.0))

        # Set the noise parameters
        self._bias: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._noise_density = config.get("noise_density", 0.4e-3)  # gauss / sqrt(hz)
        self._random_walk = config.get("random_walk", 6.4e-6)  # gauss * sqrt(hz)
        self._bias_correlation_time = config.get("bias_correlation_time", 6.0e2)  # s

        # Initial state measured by the Magnetometer
        self._state = {"magnetic_field": np.zeros((3,))}

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state

    @Sensor.update_at_rate
    def update(self, state: State, dt: float):
        """Method that implements the logic of a magnetometer. In this method we start by computing the projection
        of the vehicle body frame such in the elipsoidal model of the earth in order to get its current latitude and 
        longitude. From here the declination and inclination are computed and used to get the strength of the magnetic
        field, expressed in the inertial frame of reference (in ENU convention). This magnetic field is then rotated 
        to the body frame such that it becomes expressed in a FRD body frame relative to a NED inertial reference frame.
        (The convention adopted by PX4). Random noise and bias are added to this magnetic field.

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        # Get the latitude and longitude from the current state
        latitude, longitude = reprojection(state.position, np.radians(self._origin_lat), np.radians(self._origin_lon))

        # Magnetic declination and inclination (radians)
        declination_rad: float = np.radians(get_mag_declination(np.degrees(latitude), np.degrees(longitude)))
        inclination_rad: float = np.radians(get_mag_inclination(np.degrees(latitude), np.degrees(longitude)))

        # Compute the magnetic strength (10^5xnanoTesla)
        strength_ga: float = 0.01 * get_mag_strength(np.degrees(latitude), np.degrees(longitude))

        # Compute the Magnetic filed components according to: http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
        H: float = strength_ga * np.cos(inclination_rad)
        Z: float = np.tan(inclination_rad) * H
        X: float = H * np.cos(declination_rad)
        Y: float = H * np.sin(declination_rad)

        # Magnetic field of a body following a front-left-up (FLU) convention expressed in a East-North-Up (ENU) inertial frame
        magnetic_field_inertial: np.ndarray = np.array([X, Y, Z])

        # Rotate the magnetic field vector such that it expresses a field of a body frame according to the front-right-down (FRD)
        # expressed in a North-East-Down (NED) inertial frame (the standard used in magnetometer units)
        attitude_flu_enu = Rotation.from_quat(state.attitude)

        # Rotate the magnetic field from the inertial frame to the body frame of reference according to the FLU frame convention
        rot_body_to_world = rot_ENU_to_NED * attitude_flu_enu * rot_FLU_to_FRD.inv()

        # The magnetic field expressed in the body frame according to the front-right-down (FRD) convention
        magnetic_field_body = rot_body_to_world.inv().apply(magnetic_field_inertial)

        # -------------------------------
        # Add noise to the magnetic field
        # -------------------------------
        tau = self._bias_correlation_time

        # Discrete-time standard deviation equivalent to an "integrating" sampler with integration time dt.
        sigma_d: float = 1 / np.sqrt(dt) * self._noise_density
        sigma_b: float = self._random_walk

        # Compute exact covariance of the process after dt [Maybeck 4-114].
        sigma_b_d: float = np.sqrt(-sigma_b * sigma_b * tau / 2.0 * (np.exp(-2.0 * dt / tau) - 1.0))

        # Compute state-transition.
        phi_d: float = np.exp(-1.0 / tau * dt)

        # Add the noise to the magnetic field
        magnetic_field_noisy: np.ndarray = np.zeros((3,))
        for i in range(3):
            self._bias[i] = phi_d * self._bias[i] + sigma_b_d * np.random.randn()
            magnetic_field_noisy[i] = magnetic_field_body[i] + sigma_d * np.random.randn() + self._bias[i]

        # Add the values to the dictionary and return it
        self._state = {"magnetic_field": [magnetic_field_noisy[0], magnetic_field_noisy[1], magnetic_field_noisy[2]]}

        return self._state

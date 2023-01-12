#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
File: magnetometer.py
Author: Marcelo Jacinto
Email: marcelo.jacinto@tecnico.ulisboa.pt
Github: https://github.com/marcelojacinto
Description:
    Simulates a magnetometer. Based on the original implementation provided
    in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo) by Elia Tarasov <elias.tarasov@gmail.com>
"""
import numpy as np
from scipy.spatial.transform import Rotation

from pegasus_isaac.logic.state import State
from pegasus_isaac.logic.rotations import rot_ENU_to_NED, rot_FLU_to_FRD
from pegasus_isaac.logic.sensors.geo_mag_utils import get_mag_declination, get_mag_inclination, get_mag_strength, reprojection

class Magnetometer:

    def __init__(self, origin_latitude: float, origin_longitude: float):
        
        # Update the groundtruth latitude and longitude (in radians)
        self._origin_latitude = np.radians(origin_latitude)
        self._origin_longitude = np.radians(origin_longitude)
        
        # Set the noise parameters
        self._bias: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._noise_density: float = 0.4E-3 # gauss / sqrt(hz)
        self._random_walk: float = 6.4E-6   # gauss * sqrt(hz)
        self._bias_correlation_time: float = 6.0E2 # s

        # Initial state measured by the Magnetometer
        self._state = {'magnetic_field': np.zeros((3,))}
        
    @property
    def state(self):
        return self._state

    def update(self, state: State, dt: float):

        # Attitude of a FLU frame with respect to a ENU inertial frame, expressed in the ENU inertial frame
        state.attitude = np.array([0.0, 0.0, 0.0, 1.0])

        # Get the latitude and longitude from the current state
        latitude, longitude = reprojection(state.position, self._origin_latitude, self._origin_longitude)

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
        sigma_b_d: float = np.sqrt( - sigma_b * sigma_b * tau / 2.0 * (np.exp(-2.0 * dt / tau) - 1.0))
        
        # Compute state-transition.
        phi_d: float = np.exp(-1.0 / tau * dt)
        
        # Add the noise to the magnetic field
        magnetic_field_noisy: np.ndarray = np.zeros((3,))
        for i in range(3):
            self._bias[i] = phi_d * self._bias[i] + sigma_b_d * np.random.randn()
            magnetic_field_noisy[i] = magnetic_field_body[i] + 0.001 * np.random.randn() #+ sigma_d * np.random.randn() #+ self._bias[i]

        # Add the values to the dictionary and return it
        self._state = {'magnetic_field': [magnetic_field_noisy[0], magnetic_field_noisy[1], magnetic_field_noisy[2]]}

        return self._state
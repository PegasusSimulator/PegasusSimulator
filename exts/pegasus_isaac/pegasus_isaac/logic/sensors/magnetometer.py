#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
File: magnetometer.py
Author: Marcelo Jacinto
Email: marcelo.jacinto@tecnico.ulisboa.pt
Github: https://github.com/marcelojacinto
Description:
    Simulates a magnetometer. Based on the implementation provided
    in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo)
    by Elia Tarasov <elias.tarasov@gmail.com>
"""
import numpy as np
from ..state import State
from .geo_mag_utils import get_mag_declination, get_mag_inclination, get_mag_strength, reprojection

class Magnetometer:

    def __init__(self, origin_latitude, origin_longitude):
        
        # Update the groundtruth latitude and longitude (in radians)
        self._origin_latitude = origin_latitude
        self._origin_longitude = origin_longitude
        
        # Set the noise parameters
        self._bias: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._noise_density: float = 0.4E-3 # gauss / sqrt(hz)
        self._random_walk: float = 6.4E-6   # gauss * sqrt(hz)
        self._bias_correlation_time: float = 6.0E2 # s
        

    def update(self, state: State, dt: float) -> dict[str, float|np.ndarray]:

        # Get the latitude and longitude from the current state
        latitude, longitude = reprojection(state.position, self._origin_latitude, self._origin_longitude)

        # Magnetic declination and inclination (radians)
        declination_rad: float = get_mag_declination(latitude * 180.0 / np.pi, longitude * 180.0 / np.pi) * np.pi / 180.0
        inclination_rad: float = get_mag_inclination(latitude * 180.0 / np.pi, longitude * 180.0 / np.pi) * np.pi / 180.0

        # Compute the magnetic strength (10^5xnanoTesla)
        strength_ga: float = 0.01 * get_mag_strength(latitude * 180.0 / np.pi, longitude * 180.0 / np.pi)
        
        # Compute the Magnetic filed components according to: http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
        H: float = strength_ga * np.cos(inclination_rad);
        Z: float = np.tan(inclination_rad) * H;
        X: float = H * np.cos(declination_rad);
        Y: float = H * np.sin(declination_rad);
        
        magnetic_field: np.ndarray = np.array([X, Y, Z])
        
        # Rotate the magnetic field vector according to the vehicle orientation (and represent it in the body frame of the vehicle)
        # TODO
        magnetic_field_body = magnetic_field
        
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
            magnetic_field_noisy[i] = magnetic_field_body[i] + self._bias[i] + sigma_d * np.random.randn()

        # Add the values to the dictionary and return it
        return {'magnetic_field': magnetic_field_noisy}
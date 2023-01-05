#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
File: gps.py
Author: Marcelo Jacinto
Email: marcelo.jacinto@tecnico.ulisboa.pt
Github: https://github.com/marcelojacinto
Description:
    Simulates a gps. Based on the implementation provided
    in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo)
    by:
        Amy Wagoner <arwagoner@gmail.com> 
        Nuno Marques <nuno.marques@dronesolutions.io>
"""
import numpy as np
from .geo_mag_utils import reprojection

# TODO - Introduce delay on the GPS data

class GPS:

    def __init__(self, origin_latitude: float, origin_longitude: float, origin_altitude: float):
        
        # Define the origin's latitude, longitude and altitude corresponding to the point [0.0, 0.0, 0.0] in the inertial frame
        self._origin_latitude: float = origin_latitude
        self._origin_longitude: float = origin_longitude
        self._origin_altitude: float = origin_altitude
        
        # Parameters for GPS random walk
        self._random_walk_gps: float = np.array([0.0, 0.0, 0.0])
        self._gps_xy_random_walk: float = 2.0 # (m/s) / sqrt(hz)
        self._gps_z_random_walk: float = 4.0  # (m/s) / sqrt(hz)

        # Parameters for the position noise
        self._noise_gps_pos: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._gps_xy_noise_density: float = 2.0E-4 # (m) / sqrt(hz)
        self._gps_z_noise_density: float = 4.0E-4  # (m) / sqrt(hz)

        # Parameters for the velocity noise
        self._noise_gps_vel: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._gps_vxy_noise_density: float = 0.2  # (m/s) / sqrt(hz)
        self._gps_vz_noise_density: float = 0.4   # (m/s) / sqrt(hz)

        # Parameters for the GPS bias
        self._gps_bias: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._gps_correlation_time: float = 60

    
    def update(self, state: np.ndarray, dt: float):

        # Update noise parameters
        self._random_walk_gps[0] = self._gps_xy_random_walk * np.sqrt(dt) * np.random.randn()
        self._random_walk_gps[1] = self._gps_xy_random_walk * np.sqrt(dt) * np.random.randn()
        self._random_walk_gps[2] = self._gps_z_random_walk * np.sqrt(dt) * np.random.randn()

        self._noise_gps_pos[0] = self._gps_xy_noise_density * np.sqrt(dt) * np.random.randn()
        self._noise_gps_pos[1] = self._gps_xy_noise_density * np.sqrt(dt) * np.random.randn()
        self._noise_gps_pos[2] = self._gps_z_noise_density * np.sqrt(dt) * np.random.randn()
        
        self._noise_gps_vel[0] = self._gps_vxy_noise_density * np.sqrt(dt) * np.random.randn()
        self._noise_gps_vel[1] = self._gps_vxy_noise_density * np.sqrt(dt) * np.random.randn()
        self._noise_gps_vel[2] = self._gps_vz_noise_density * np.sqrt(dt) * np.random.randn()

        # Perform GPS bias integration (using euler integration -> to be improved)
        self._gps_bias[0] = self._gps_bias[0] + self._random_walk_gps[0] * dt - self._gps_bias[0] / self._gps_correlation_time
        self._gps_bias[1] = self._gps_bias[1] + self._random_walk_gps[1] * dt - self._gps_bias[1] / self._gps_correlation_time
        self._gps_bias[2] = self._gps_bias[2] + self._random_walk_gps[2] * dt - self._gps_bias[2] / self._gps_correlation_time

        # reproject position with noise into geographic coordinates
        pos_with_noise: np.ndarray = state.position + self._noise_gps_pos + self._gps_bias
        latitude, longitude = reprojection(pos_with_noise, self._origin_latitude, self._origin_longitude)

        # Add noise to the velocity expressed in the world frame
        velocity: np.ndarray = state.linear_velocity + self._noise_gps_vel

        # Compute the xy speed
        speed: float = np.linalg.norm(velocity[:2])

        # Add the values to the dictionary and return it
        return {
            'latitude': latitude * 180.0 / np.pi, 
            'longitude': longitude * 180.0 / np.pi, 
            'altitude': state.position[2] + self._origin_altitude - self._noise_gps_pos[2] + self._gps_bias[2],
            'eph': 1.0, 
            'epv': 1.0, 
            'speed': speed, 
            'velocity_north': velocity[0], 
            'velocity_east': velocity[1], 
            'velocity_down': -velocity[2]}
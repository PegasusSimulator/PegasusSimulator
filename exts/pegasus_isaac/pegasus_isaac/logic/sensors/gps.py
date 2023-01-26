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
__all__ = ["GPS"]

import numpy as np
from pegasus_isaac.logic.sensors import Sensor
from pegasus_isaac.logic.sensors.geo_mag_utils import reprojection

# TODO - Introduce delay on the GPS data

class GPS(Sensor):

    def __init__(self, config={}):
        """
        Constructor for a GPS sensor. Receives as arguments the:
        origin_latitude: float with the latitude of the inertial frame origin in degrees
        origin_longitude: float with the longitude of the inertial frame origin in degrees
        origin_altitude: float with the base altitude of the inertial frame origin in meters
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="GPS", update_rate=config.get("update_rate", 250.0))

        # Define the GPS simulated/fixed values
        self._fix_type = config.get("fix_type", 3)
        self._eph = config.get("eph", 1.0)
        self._epv = config.get("epv", 1.0)
        self._sattelites_visible = config.get("sattelites_visible", 10)
        
        # Parameters for GPS random walk
        self._random_walk_gps = np.array([0.0, 0.0, 0.0])
        self._gps_xy_random_walk = config.get("gps_xy_random_walk", 2.0)   # (m/s) / sqrt(hz)
        self._gps_z_random_walk = config.get("gps_z_random_walk", 4.0)     # (m/s) / sqrt(hz)

        # Parameters for the position noise
        self._noise_gps_pos = np.array([0.0, 0.0, 0.0])
        self._gps_xy_noise_density = config.get("gps_xy_noise_density", 2.0E-4) # (m) / sqrt(hz)
        self._gps_z_noise_density = config.get("gps_z_noise_density", 4.0E-4)   # (m) / sqrt(hz)

        # Parameters for the velocity noise
        self._noise_gps_vel = np.array([0.0, 0.0, 0.0])
        self._gps_vxy_noise_density = config.get("gps_vxy_noise_density", 0.2)  # (m/s) / sqrt(hz)
        self._gps_vz_noise_density = config.get("gps_vz_noise_density", 0.4)    # (m/s) / sqrt(hz)

        # Parameters for the GPS bias
        self._gps_bias = np.array([0.0, 0.0, 0.0])
        self._gps_correlation_time = config.get("gps_correlation_time", 60)

        # Save the current state measured by the GPS (and initialize at the origin)
        self._state = {
            'latitude': np.radians(self._origin_lat), 
            'longitude': np.radians(self._origin_lon), 
            'altitude': self._origin_alt,
            'eph': 1.0, 
            'epv': 1.0, 
            'speed': 0.0, 
            'velocity_north': 0.0, 
            'velocity_east': 0.0, 
            'velocity_down': 0.0,
            # Constant values
            'fix_type': self._fix_type,
            'eph': self._eph,
            'epv': self._epv,
            'cog': 0.0,
            'sattelites_visible': self._sattelites_visible,
            'latitude_gt': np.radians(self._origin_lat),
            'longitude_gt': np.radians(self._origin_lon),
            'altitude_gt': self._origin_alt
        }

    @property
    def state(self):
        return self._state
    
    @Sensor.update_at_rate
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
        pos_with_noise: np.ndarray = state.position + self._noise_gps_pos # + self._gps_bias
        latitude, longitude = reprojection(pos_with_noise, np.radians(self._origin_lat), np.radians(self._origin_lon))

        # Compute the values of the latitude and longitude without noise (for groundtruth measurements)
        latitude_gt, longitude_gt = reprojection(state.position, np.radians(self._origin_lat), np.radians(self._origin_lon))

        # Add noise to the velocity expressed in the world frame
        velocity: np.ndarray = state.linear_velocity #+ self._noise_gps_vel

        # Compute the xy speed
        speed: float = np.linalg.norm(velocity[:2])

        # Course over ground (NOT heading, but direction of movement), 
        # 0.0..359.99 degrees. If unknown, set to: 65535 [cdeg] (type:uint16_t)
        ve = velocity[0]
        vn = velocity[1]
        cog = np.degrees(np.arctan2(ve, vn))
        
        if cog < 0.0:
            cog = cog + 360.0

        cog = cog * 100

        # Add the values to the dictionary and return it
        self._state = {
            'latitude': np.degrees(latitude), 
            'longitude': np.degrees(longitude), 
            'altitude': state.position[2] + self._origin_alt - self._noise_gps_pos[2] + self._gps_bias[2],
            'eph': 1.0, 
            'epv': 1.0, 
            'speed': speed, 
            # Conversion from ENU (standard of Isaac Sim to NED - used in GPS sensors)
            'velocity_north': velocity[1], 
            'velocity_east': velocity[0], 
            'velocity_down': -velocity[2],
            # Constant values
            'fix_type': self._fix_type,
            'eph': self._eph,
            'epv': self._epv,
            'cog': 0.0, #cog,
            'sattelites_visible': self._sattelites_visible,
            'latitude_gt': latitude_gt,
            'longitude_gt': longitude_gt, 
            'altitude_gt': state.position[2] + self._origin_alt
        }

        return self._state
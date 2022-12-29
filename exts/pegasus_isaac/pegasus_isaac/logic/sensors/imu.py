#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
File: imu.py
Author: Marcelo Jacinto
Email: marcelo.jacinto@tecnico.ulisboa.pt
Github: https://github.com/marcelojacinto
Description:
    Simulates an imu. Based on the implementation provided
    in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo)
"""
import numpy as np
from pyquaternion import Quaternion

from ..state import State
from .geo_mag_utils import GRAVITY_VECTOR

class IMU:

    def __init__(self):
        
        # Orientation noise constant
        self._orientation_noise: float = 0.0
        
        # Gyroscope noise constants
        self._gyroscope_bias: np.ndarray = np.zeros((3,))
        self._gyroscope_noise_density: float = 0.0
        self._gyroscope_random_walk: float = 0.0
        self._gyroscope_bias_correlation_time: float = 0.0
        self._gyroscope_turn_on_bias_sigma: float = 0.0
        
        # Accelerometer noise constants
        self._accelerometer_bias: np.ndarray = np.zeros((3,))
        self._accelerometer_noise_density: float = 0.0
        self._accelerometer_random_walk: float = 0.0
        self._accelerometer_bias_correlation_time: float = 0.0
        self._accelerometer_turn_on_bias_sigma: float = 0.0


    def update(self, state: State, dt: float) -> dict[str, float|np.ndarray]:  
        
        # Gyroscopic terms
        tau_g: float = self._accelerometer_bias_correlation_time
        
        # Discrete-time standard deviation equivalent to an "integrating" sampler with integration time dt
        sigma_g_d: float = 1 / np.sqrt(dt) * self._gyroscope_noise_density 
        sigma_b_g: float = self._gyroscope_random_walk
  
        # Compute exact covariance of the process after dt [Maybeck 4-114]
        sigma_b_g_d: float = np.sqrt(- sigma_b_g * sigma_b_g * tau_g / 2.0 * (np.exp(-2.0 * dt / tau_g) - 1.0))
        
        # Compute state-transition
        phi_g_d: float = np.exp(-1.0 / tau_g * dt)
        
        # Simulate gyroscope noise processes and add them to the true angular rate.
        angular_velocity: np.ndarray = np.zeros((3,))
        
        for i in range(3):
            self._gyroscope_bias[i] = phi_g_d * self._gyroscope_bias[i] + sigma_b_g_d * np.random.randn()
            angular_velocity[i] = state.angular_velocity[i] + self._gyroscope_bias[i] + sigma_g_d * np.random.randn()
        
        # Get the attitude of the vehicle
        tau_a: float = self._accelerometer_bias_correlation_time
        
        # Discrete-time standard deviation equivalent to an "integrating" sampler with integration time dt
        sigma_a_d: float = 1.0 / np.sqrt(dt) * self._accelerometer_noise_density
        sigma_b_a: float = self._accelerometer_random_walk
        
        # Compute exact covariance of the process after dt [Maybeck 4-114].
        sigma_b_a_d: float = np.sqrt( - sigma_b_a * sigma_b_a * tau_a / 2.0 * (np.exp(-2.0 * dt / tau_a) - 1.0))
  
        # Compute state-transition.
        phi_a_d: float = np.exp(-1.0 / tau_a * dt)
        
        # Simulate the accelerometer noise processes and add them to the true linear aceleration values
        linear_acceleration: np.ndarray = np.zeros((3,))
        
        for i in range(3):
            self._accelerometer_bias[i] = phi_a_d * self._accelerometer_bias[i] + sigma_b_a_d * np.random.rand()
            linear_acceleration[i] = state.linear_acceleration[i] + self._accelerometer_bias[i] + sigma_a_d * np.random.randn()
    
        # Create a small "noisy" rotation about each axis x, y and z and generate a rotation from that noise
        noise_x: Quaternion = Quaternion(axis=[1, 0, 0], angle=np.random.rand() *  self._orientation_noise)
        noise_y: Quaternion = Quaternion(axis=[0, 1, 0], angle=np.random.rand() *  self._orientation_noise)
        noise_z: Quaternion = Quaternion(axis=[0, 0, 1], angle=np.random.rand() *  self._orientation_noise)
        noise_quaternion: Quaternion = noise_x * noise_y * noise_z
    
        # Simulate the attitude affected by noise
        attitude: Quaternion = state.attitude * noise_quaternion

        # Add the values to the dictionary and return it
        return {'orientation': attitude, 'angular_velocity': angular_velocity, 'linear_acceleration': linear_acceleration}

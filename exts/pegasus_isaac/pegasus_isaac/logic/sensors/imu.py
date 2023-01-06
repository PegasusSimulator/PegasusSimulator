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

from ..state import State

from omni.isaac.core.utils.rotations import euler_angles_to_quat


class IMU:

    def __init__(self):
        
        # Orientation noise constant
        self._orientation_noise: float = 0.0
        
        # Gyroscope noise constants
        self._gyroscope_bias: np.ndarray = np.zeros((3,))
        self._gyroscope_noise_density: float = 2.0 * 35.0 / 3600.0 / 180.0 * np.pi
        self._gyroscope_random_walk: float = 2.0 * 4.0 / 3600.0 / 180.0 * np.pi
        self._gyroscope_bias_correlation_time: float = 1.0E3
        self._gyroscope_turn_on_bias_sigma: float = 0.5 / 180.0 * np.pi
        
        # Accelerometer noise constants
        self._accelerometer_bias: np.ndarray = np.zeros((3,))
        self._accelerometer_noise_density: float = 2.0 * 2.0E-3
        self._accelerometer_random_walk: float = 2.0 * 3.0E-3
        self._accelerometer_bias_correlation_time: float = 300.0
        self._accelerometer_turn_on_bias_sigma: float = 20.0E-3 * 9.8

        # Save the current state measured by the IMU
        self._state = {
            'orientation': np.array([1.0, 0.0, 0.0, 0.0 ]), 
            'angular_velocity': np.array([0.0, 0.0, 0.0]), 
            'linear_acceleration': np.array([0.0, 0.0, 0.0])
        }

    @property
    def state(self):
        return self._state

    def update(self, state: State, dt: float):  
        
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
        # TODO - add noise to the attitude
        #noise_x = euler_angles_to_quat(euler_angles=[np.random.randn() * self._orientation_noise, 0.0, 0.0], degrees=False)
        #noise_y= euler_angles_to_quat(euler_angles=[0.0, np.random.randn() * self._orientation_noise, 0.0], degrees=False)
        #noise_z= euler_angles_to_quat(euler_angles=[0.0, 0.0, np.random.randn() * self._orientation_noise], degrees=False)
        #noise_quaternion = noise_x * noise_y * noise_z
    
        # Simulate the attitude affected by noise
        attitude = state.attitude #* noise_quaternion

        # Add the values to the dictionary and return it
        self._state = {
            'orientation': attitude, 
            'angular_velocity': angular_velocity, 
            'linear_acceleration': linear_acceleration
        }

        return self._state

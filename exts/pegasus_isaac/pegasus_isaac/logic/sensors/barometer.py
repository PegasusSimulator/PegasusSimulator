#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
File: barometer.py
Author: Marcelo Jacinto
Email: marcelo.jacinto@tecnico.ulisboa.pt
Github: https://github.com/marcelojacinto
Description:
    Simulates a barometer. Based on the implementation provided
    in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo)
    by Elia Tarasov <elias.tarasov@gmail.com>.
    
    References:
    [1] A brief summary of atmospheric modeling with citations:
    Cavcar, M., http://fisicaatmo.at.fcen.uba.ar/practicas/ISAweb.pdf
"""
import numpy as np
from ..state import State
from .geo_mag_utils import GRAVITY_VECTOR

class Barometer:

    def __init__(self, noise: float, z_start: float, altitude_home: float):
        
        self._noise: float = noise
        self._z_start: float = z_start
        self._altitude_home: float = altitude_home

        # Define the constants for the barometer   
        self._TEMPERATURE_MSL: float = 288.15
        self._PRESSURE_MSL: float = 101325.0
        self._LAPSE_RATE: float = 0.0065
        self._AIR_DENSITY_MSL: float = 1.225
        self._ABSOLUTE_ZERO_C: float = -273.15

        # Auxiliar variables for generating the noise
        self._baro_rnd_use_last: bool = False
        self._baro_rnd_y2: float = 0.0
        self._baro_drift_pa: float = 0.0
        self._baro_drift_pa_per_sec: float = 0.0
        
        # The gravity acceleration
        self._gravity: float = GRAVITY_VECTOR[2] 


    def update(self, state: State, dt: float) -> dict[str, float | np.ndarray]:

        # Compute the temperature at the current altitude
        # Inverting the sign, because we are using NED convention
        # and the z is negative upwards
        alt_rel: float = -(state.position[2] - self._z_start)
        alt_amsl: float = self._altitude_home - alt_rel
        temperature_local: float = self._TEMPERATURE_MSL - self._LAPSE_RATE * alt_amsl

        # Compute the absolute pressure at local temperature
        pressure_ratio: float = np.pow(self._TEMPERATURE_MSL / temperature_local, 5.256)
        absolute_pressure: float = self._PRESSURE_MSL / pressure_ratio
        
        # Generate a Gaussian noise sequence using polar form of Box-Muller transformation
        if not self._baro_rnd_use_last:
            
            w: float = 1.0
            
            while w >= 1.0:
                x1: float = 2.0 * np.random.randn() - 1.0
                x2: float = 2.0 * np.random.randn() - 1.0
                w = x1 * x1 + x2 * x2

            w = np.sqrt((-2.0 * np.log(w)) / w)
            y1: float = x1 * w
            self._baro_rnd_y2 = x2 * w
            self._baro_rnd_use_last = True
        else:
            y1: float = self._baro_rnd_y2
            self._baro_rnd_use_last = False

        # Apply noise and drift
        abs_pressure_noise: float = y1
        self._baro_drift_pa = self._baro_drift_pa + (self._baro_drift_pa_per_sec * dt)
        absolute_pressure_noisy: float = absolute_pressure + abs_pressure_noise + self._baro_drift_pa

        # Convert to hPa
        absolute_pressure_noisy_hpa: float = absolute_pressure_noisy * 0.01

        # Compute air density at local temperature
        density_ratio: float = np.pow(self._TEMPERATURE_MSL / temperature_local, 4.256)
        air_density: float = self._AIR_DENSITY_MSL / density_ratio

        # Compute pressure altitude including effect of pressure noise
        pressure_altitude: float = alt_amsl - (abs_pressure_noise + self._baro_drift_pa) / (self._gravity * air_density)

        # Compute temperature in celsius
        temperature_celsius: float = temperature_local + self._ABSOLUTE_ZERO_C

        # Add the values to the dictionary and return it
        return {'absolute_pressure': absolute_pressure_noisy_hpa, 'pressure_altitude': pressure_altitude, 'temperature': temperature_celsius}
        
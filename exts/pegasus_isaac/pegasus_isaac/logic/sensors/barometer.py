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
__all__ = ["Barometer", "BarometerConfig"]

import numpy as np
from pegasus_isaac.logic.state import State
from pegasus_isaac.logic.sensors import Sensor
from pegasus_isaac.logic.sensors.geo_mag_utils import GRAVITY_VECTOR

DEFAULT_HOME_ALT_AMSL = 488.0

class BarometerConfig:

    def __init__(self):
        self.temperature_msl: float = 288.15   # temperature at MSL [K] (15 [C])
        self.pressure_msl: float = 101325.0    # pressure at MSL [Pa]
        self.lapse_rate: float =  0.0065       # reduction in temperature with altitude for troposphere [K/m]
        self.air_density_msl: float = 1.225    # air density at MSL [kg/m^3]
        self.absolute_zero_c: float = -273.15  # [C]
        self.update_rate: float = 250.0        # [Hz]

    def load_from_dict(self, data: dict):
        """
        Method used to load/generate a BarometerConfig object given a set of parameters read from a dictionary 
        """
        self.temperature_msl = data.get("temperature_msl", self.temperature_msl)
        self.pressure_msl = data.get("pressure_msl", self.pressure_msl)
        self.lapse_rate = data.get("lapse_rate", self.lapse_rate)
        self.air_density_msl = data.get("air_density_msl", self.air_density_msl)
        self.absolute_zero_c = data.get("absolute_zero_c", self.absolute_zero_c)
        self.update_rate = data.get("update_rate", self.update_rate)

    def get_sensor_from_config(self):
        return Barometer(self)

class Barometer(Sensor):

    def __init__(self, config=BarometerConfig()):

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="Barometer", update_rate=config.update_rate)
        
        self._z_start: float = None

        # Setup the default home altitude (aka the altitude at the [0.0, 0.0, 0.0] coordinate on the simulated world)
        # If desired, the user can override this default by calling the initialize() method defined inside the Sensor
        # implementation
        self._origin_alt = DEFAULT_HOME_ALT_AMSL

        # Define the constants for the barometer   
        # International standard atmosphere (troposphere model - valid up to 11km) see [1]
        self._TEMPERATURE_MSL: float = config.temperature_msl   # temperature at MSL [K] (15 [C])
        self._PRESSURE_MSL: float = config.pressure_msl         # pressure at MSL [Pa]
        self._LAPSE_RATE: float = config.lapse_rate             # reduction in temperature with altitude for troposphere [K/m]
        self._AIR_DENSITY_MSL: float = config.air_density_msl   # air density at MSL [kg/m^3]
        self._ABSOLUTE_ZERO_C: float = config.absolute_zero_c   # [C]

        # Auxiliar variables for generating the noise
        self._baro_rnd_use_last: bool = False
        self._baro_rnd_y2: float = 0.0
        self._baro_drift_pa: float = 0.0
        self._baro_drift_pa_per_sec: float = 0.0

        # Save the current state measured by the Baramoter
        self._state = {'absolute_pressure': 0.0, 'pressure_altitude': 0.0, 'temperature': 0.0}

    @property
    def state(self):
        return self._state

    @Sensor.update_at_rate
    def update(self, state: State, dt: float):

        # Set the initial altitude if not yet defined
        if self._z_start is None:
            self._z_start = state.position[2]

        # Compute the temperature at the current altitude
        alt_rel: float = state.position[2] - self._z_start
        alt_amsl: float = self._origin_alt + alt_rel
        temperature_local: float = self._TEMPERATURE_MSL - self._LAPSE_RATE * alt_amsl

        # Compute the absolute pressure at local temperature
        pressure_ratio: float = np.power(self._TEMPERATURE_MSL / temperature_local, 5.256)
        absolute_pressure: float = self._PRESSURE_MSL / pressure_ratio
        
        # Generate a Gaussian noise sequence using polar form of Box-Muller transformation
        if not self._baro_rnd_use_last:
            
            w: float = 1.0
            
            while w >= 1.0:
                x1: float = 2.0 * np.random.randn() - 1.0
                x2: float = 2.0 * np.random.randn() - 1.0
                w = (x1 * x1) + (x2 * x2)

            w = np.sqrt((-2.0 * np.log(w)) / w)
            y1: float = x1 * w
            self._baro_rnd_y2 = x2 * w
            self._baro_rnd_use_last = True
        else:
            y1: float = self._baro_rnd_y2
            self._baro_rnd_use_last = False

        # Apply noise and drift
        abs_pressure_noise: float = y1  # 1 Pa RMS noise
        self._baro_drift_pa = self._baro_drift_pa + (self._baro_drift_pa_per_sec * dt)
        absolute_pressure_noisy: float = absolute_pressure + abs_pressure_noise + self._baro_drift_pa

        # Convert to hPa
        absolute_pressure_noisy_hpa: float = absolute_pressure_noisy * 0.01

        # Compute air density at local temperature
        density_ratio: float = np.power(self._TEMPERATURE_MSL / temperature_local, 4.256)
        air_density: float = self._AIR_DENSITY_MSL / density_ratio

        # Compute pressure altitude including effect of pressure noise
        #pressure_altitude: float = alt_amsl - (abs_pressure_noise + self._baro_drift_pa) / (np.linalg.norm(GRAVITY_VECTOR) * air_density)
        pressure_altitude: float = alt_amsl - (abs_pressure_noise) / (np.linalg.norm(GRAVITY_VECTOR) * air_density)

        # Compute temperature in celsius
        temperature_celsius: float = temperature_local + self._ABSOLUTE_ZERO_C

        # Add the values to the dictionary and return it
        self._state = {
            'absolute_pressure': absolute_pressure_noisy_hpa, 
            'pressure_altitude': pressure_altitude, 
            'temperature': temperature_celsius
        }

        return self._state
        
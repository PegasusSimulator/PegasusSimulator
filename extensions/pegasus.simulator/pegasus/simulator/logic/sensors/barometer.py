"""
| File: barometer.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Simulates a barometer. Based on the implementation provided in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo) by Elia Tarasov.
| References: Both the original implementation provided in the gazebo based simulation and this one are based on the following article - 'A brief summary of atmospheric modeling', Cavcar, M., http://fisicaatmo.at.fcen.uba.ar/practicas/ISAweb.pdf
"""
__all__ = ["Barometer"]

import numpy as np
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.sensors.geo_mag_utils import GRAVITY_VECTOR

DEFAULT_HOME_ALT_AMSL = 488.0

class Barometer(Sensor):
    """The class that implements a barometer sensor. This class inherits the base class Sensor.
    """

    def __init__(self, config={}):
        """Initialize the Barometer class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the Barometer - it can be empty or only have some of the parameters used by the Barometer.
            
        Examples:
            The dictionary default parameters are

            >>> {"temperature_msl": 288.15,           # temperature at MSL [K] (15 [C])
            >>>  "pressure_msl": 101325.0,            # pressure at MSL [Pa]
            >>>  "lapse_rate": 0.0065,                # reduction in temperature with altitude for troposphere [K/m]
            >>>  "air_density_msl": 1.225,            # air density at MSL [kg/m^3]
            >>>  "absolute_zero": -273.15,            # [C]
            >>>  "drift_pa_per_sec": 0.0,             # Pa
            >>>  "update_rate": 250.0}                # Hz
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="Barometer", update_rate=config.get("update_rate", 250.0))

        self._z_start: float = None

        # Setup the default home altitude (aka the altitude at the [0.0, 0.0, 0.0] coordinate on the simulated world)
        # If desired, the user can override this default by calling the initialize() method defined inside the Sensor
        # implementation
        self._origin_alt = DEFAULT_HOME_ALT_AMSL

        # Define the constants for the barometer
        # International standard atmosphere (troposphere model - valid up to 11km) see [1]
        self._TEMPERATURE_MSL: float = config.get("temperature_msl", 288.15)  # temperature at MSL [K] (15 [C])
        self._PRESSURE_MSL: float = config.get("pressure_msl", 101325.0)  # pressure at MSL [Pa]
        self._LAPSE_RATE: float = config.get(
            "lapse_rate", 0.0065
        )  # reduction in temperature with altitude for troposphere [K/m]
        self._AIR_DENSITY_MSL: float = config.get("air_density_msl", 1.225)  # air density at MSL [kg/m^3]
        self._ABSOLUTE_ZERO_C: float = config.get("absolute_zero", -273.15)  # [C]

        # Set the drift for the sensor
        self._baro_drift_pa_per_sec: float = config.get("drift_pa_per_sec", 0.0)

        # Auxiliar variables for generating the noise
        self._baro_rnd_use_last: bool = False
        self._baro_rnd_y2: float = 0.0
        self._baro_drift_pa: float = 0.0
        

        # Save the current state measured by the Baramoter
        self._state = {"absolute_pressure": 0.0, "pressure_altitude": 0.0, "temperature": 0.0}

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state

    @Sensor.update_at_rate
    def update(self, state: State, dt: float):
        """Method that implements the logic of a barometer. In this method we compute the relative altitude of the vehicle
        relative to the origin's altitude. Aditionally, we compute the actual altitude of the vehicle, local temperature and
        absolute presure, based on the reference - [A brief summary of atmospheric modeling, Cavcar, M., http://fisicaatmo.at.fcen.uba.ar/practicas/ISAweb.pdf]

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        # Set the initial altitude if not yet defined
        if self._z_start is None:
            self._z_start = state.position[2]

        # Compute the temperature at the current altitude
        alt_rel: float = state.position[2] - self._z_start
        alt_amsl: float = self._origin_alt + alt_rel
        temperature_local: float = self._TEMPERATURE_MSL - self._LAPSE_RATE * alt_amsl

        # Compute the absolute pressure at local temperature
        pressure_ratio: float = np.power(self._TEMPERATURE_MSL / temperature_local, 5.2561)
        absolute_pressure: float = self._PRESSURE_MSL / pressure_ratio

        # Generate a Gaussian noise sequence using polar form of Box-Muller transformation
        # Honestly, this is overkill and will get replaced by numpys random.randn. 
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
        self._baro_drift_pa = self._baro_drift_pa + (self._baro_drift_pa_per_sec * dt)  # Update the drift
        absolute_pressure_noisy: float = absolute_pressure + abs_pressure_noise + self._baro_drift_pa_per_sec

        # Convert to hPa (Note: 1 hPa = 100 Pa)
        absolute_pressure_noisy_hpa: float = absolute_pressure_noisy * 0.01

        # Compute air density at local temperature
        density_ratio: float = np.power(self._TEMPERATURE_MSL / temperature_local, 4.256)
        air_density: float = self._AIR_DENSITY_MSL / density_ratio

        # Compute pressure altitude including effect of pressure noise
        pressure_altitude: float = alt_amsl - (abs_pressure_noise + self._baro_drift_pa) / (np.linalg.norm(GRAVITY_VECTOR) * air_density)
        #pressure_altitude: float = alt_amsl - (abs_pressure_noise) / (np.linalg.norm(GRAVITY_VECTOR) * air_density)

        # Compute temperature in celsius
        temperature_celsius: float = temperature_local + self._ABSOLUTE_ZERO_C

        # Add the values to the dictionary and return it
        self._state = {
            "absolute_pressure": absolute_pressure_noisy_hpa,
            "pressure_altitude": pressure_altitude,
            "temperature": temperature_celsius,
        }

        return self._state

"""
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Sensor class which is used as the base for all the sensors.
"""
__all__ = ["Sensor"]

from pegasus.simulator.logic.state import State

class Sensor:
    """The base class for implementing a sensor

    Attributes:
        update_period (float): The period for each sensor update: update_period = 1 / update_rate (in s).
        origin_lat (float): The latitude of the origin of the world in degrees (might get used by some sensors).
        origin_lon (float): The longitude of the origin of the world in degrees (might get used by some sensors).
        origin_alt (float): The altitude of the origin of the world relative to sea water level (might get used by some sensors)
    """
    def __init__(self, sensor_type: str, update_rate: float):
        """Initialize the Sensor class

        Args:
            sensor_type (str): A name that describes the type of sensor
            update_rate (float): The rate at which the data in the sensor should be refreshed (in Hz)
        """

        # Set the sensor type and update rate
        self._sensor_type = sensor_type
        self._update_rate = update_rate
        self._update_period = 1.0 / self._update_rate

        # Auxiliar variables used to control whether to update the sensor or not given the time elapsed
        self._first_update = True
        self._total_time = 0.0

        # Set the "configuration of the world" - some sensors might need it
        self._origin_lat = -999
        self._origin_lon = -999
        self._origin_alt = 0.0

    def initialize(self, origin_lat, origin_lon, origin_alt):
        """Method that initializes the sensor latitude, longitude and altitude attributes.
        
        Note:
            Given that some sensors require the knowledge of the latitude, longitude and altitude of the [0, 0, 0] coordinate
            of the world, then we might as well just save this information for whatever sensor that comes
        
        Args:
            origin_lat (float): The latitude of the origin of the world in degrees (might get used by some sensors).
            origin_lon (float): The longitude of the origin of the world in degrees (might get used by some sensors).
            origin_alt (float): The altitude of the origin of the world relative to sea water level (might get used by some sensors).
        """
        self._origin_lat = origin_lat
        self._origin_lon = origin_lon
        self._origin_alt = origin_alt

    def set_update_rate(self, update_rate: float):
        """Method that changes the update rate and period of the sensor

        Args:
            update_rate (float): The new rate at which the data in the sensor should be refreshed (in Hz)
        """
        self._update_rate = update_rate
        self._update_period = 1.0 / self._update_rate

    def update_at_rate(fnc):
        """Decorator function used to check if the time elapsed between the last sensor update call and the current 
        sensor update call is higher than the defined update_rate of the sensor. If so, we need to actually compute new
        values to simulate a measurement of the sensor at a given rate.

        Args:
            fnc (function): The function that we want to enforce a specific update rate.

        Examples:
            >>> class GPS(Sensor):
            >>>    @Sensor.update_at_rate
            >>>    def update(self):
            >>>        (do some logic here)

        Returns:
            [None, Dict]: This decorator function returns None if there was no data to be produced by the sensor at the
            specified timestamp or a dict with the current state of the sensor otherwise.
        """
        
        # 

        # Define a wrapper function so that the "self" of the object can be passed to the function as well
        def wrapper(self, state: State, dt: float):

            # Add the total time passed between the last time the sensor was updated and the current call
            self._total_time += dt

            # If it is time to update the sensor data, then just call the update function of the sensor
            if self._total_time >= self._update_period or self._first_update:

                # Result of the update function for the sensor
                result = fnc(self, state, self._total_time)

                # Reset the auxiliar counter variables
                self._first_update = False
                self._total_time = 0.0

                return result
            return None
        return wrapper

    @property
    def sensor_type(self):
        """
        (str) A name that describes the type of sensor.
        """
        return self._sensor_type

    @property
    def update_rate(self):
        """
        (float) The rate at which the data in the sensor should be refreshed (in Hz).
        """
        return self._update_rate

    @property
    def state(self):
        """
        (dict) A dictionary which contains the data produced by the sensor at any given time.
        """
        return None

    def update(self, state: State, dt: float):
        """Method that should be implemented by the class that inherits Sensor. This is where the actual implementation
        of the sensor should be performed.

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """
        pass

    def config_from_dict(self, config_dict):
        """Method that should be implemented by the class that inherits Sensor. This is where the configuration of the 
        sensor based on a dictionary input should be performed.

        Args:
            config_dict (dict): A dictionary containing the configurations of the sensor
        """
        pass

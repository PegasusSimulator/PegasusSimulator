#!/usr/bin/env python
from pegasus_isaac.logic.state import State

class Sensor:

    def __init__(self, sensor_type: str, update_rate: float):
        
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
        """
        Given that some sensors require the knowledge of the latitude, longitude and altitude of the [0, 0, 0] coordinate
        of the world, then we might as well just save this information for whatever sensor that comes
        """
        self._origin_lat = origin_lat
        self._origin_lon = origin_lon
        self._origin_alt = origin_alt

    def set_update_rate(self, update_rate: float):
        self._update_rate = update_rate
        self._update_period = 1.0 / self._update_rate

    def update_at_rate(fnc):
        """
        Decorator function used to check if the time elapsed between the last sensor update call and the current
        sensor update call is higher than the defined update_rate of the sensor. If so, we need to actually compute new
        values to simulate a measurement of the sensor at a given rate
        """

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
        return self._sensor_type
    
    @property
    def update_rate(self):
        return self._update_rate

    @property
    def state(self):
        return None

    def update(self, state: State, dt: float):
        pass

    def config_from_dict(self, config_dict):
        pass

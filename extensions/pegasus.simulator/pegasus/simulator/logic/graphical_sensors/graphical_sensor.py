"""
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Sensor class which is used as the base for all the sensors.
"""
__all__ = ["GraphicalSensor"]

import time
from pegasus.simulator.logic.state import State

class GraphicalSensor:

    """The base class for implementing a graphical sensor (such as a camera)

    Attributes:
        update_period (float): The period for each sensor update: update_period = 1 / update_rate (in s).
    """

    def __init__(self, sensor_type: str, update_rate: float):
        """
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

        self._vehicle = None

    def initialize(self, vehicle):
        """A method that can be invoked when the simulation is starting to give access to the control backend 
        to the entire vehicle object. Even though we provide update_sensor and update_state callbacks that are called
        at every physics step with the latest vehicle state and its sensor data, having access to the full vehicle
        object may prove usefull under some circumstances. This is nice to give users the possibility of overiding
        default vehicle behaviour via this control backend structure.

        Args:
            vehicle (Vehicle): A reference to the vehicle that this sensor is associated with
        """
        self._vehicle = vehicle

    def update_at_rate(fnc):
        """Decorator function used to check if the time elapsed between the last sensor update call and the current 
        sensor update call is higher than the defined update_rate of the sensor. If so, we need to actually compute new
        values to simulate a measurement of the sensor at a given rate.

        Args:
            fnc (function): The function that we want to enforce a specific update rate.

        Examples:
            >>> class Camera(GraphicalSensor):
            >>>    @GraphicalSensor.update_at_rate
            >>>    def update(self):
            >>>        (do some logic here)

        Returns:
            [None, Dict]: This decorator function returns None if there was no data to be produced by the sensor at the
            specified timestamp or a dict with the current state of the sensor otherwise.
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
    
    """
     Properties
    """
    @property
    def vehicle(self):
        """A reference to the vehicle associated with this backend.

        Returns:
            Vehicle: A reference to the vehicle associated with this backend.
        """
        return self._vehicle

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

    def start(self):
        """Method that when implemented should handle the begining of the simulation of vehicle
        """
        pass

    def stop(self):
        """Method that when implemented should handle the stopping of the simulation of vehicle
        """
        pass

    def reset(self):
        """Method that when implemented, should handle the reset of the vehicle simulation to its original state
        """
        pass

    def config_from_dict(self, config_dict):
        """Method that should be implemented by the class that inherits Sensor. This is where the configuration of the 
        sensor based on a dictionary input should be performed.

        Args:
            config_dict (dict): A dictionary containing the configurations of the sensor
        """
        pass

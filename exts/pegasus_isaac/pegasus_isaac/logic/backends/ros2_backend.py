#!/usr/bin/env python
from pegasus_isaac.logic.backends.backend import Backend

class ROS2Backend(Backend):

    def __init__(self):
        pass

    def update_sensor(self, sensor_type:str, data):
        """
        Method that when implemented, should handle the receival of sensor data
        """
        pass

    def update_state(self, state):
        """
        Method that when implemented, should handle the receivel of the state of the vehicle using this callback
        """
        pass

    def input_reference(self):
        """
        Method that when implemented, should return a list of desired angular velocities to apply to the vehicle rotors
        """
        return []

    def update(self, dt: float):
        """
        Method that when implemented, should be used to update the state of the backend and the information being sent/received
        from the communication interface. This method will be called by the simulation on every physics step
        """
        pass

    def start(self):
        """
        Method that when implemented should handle the begining of the simulation of vehicle
        """
        pass

    def stop(self):
        """
        Method that when implemented should handle the stopping of the simulation of vehicle
        """
        pass

    def reset(self):
        """
        Method that when implemented, should handle the reset of the vehicle simulation to its original state
        """
        pass
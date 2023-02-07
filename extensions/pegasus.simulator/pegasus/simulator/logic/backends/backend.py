"""
| File: backend.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description:
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""

class Backend:
    """
    This class defines the templates for the communication and control backend. Every vehicle can have at least one backend
    at the same time. Every timestep, the methods 'update_state' and 'update_sensor' are called to update the data produced
    by the simulation, i.e. for every time step the backend will receive teh current state of the vehicle and its sensors. 
    Additionally, the backend must provide a method named 'input_reference' which will be used by the vehicle simulation
    to know the desired angular velocities to apply to the rotors of the vehicle. The method 'update' is called on every
    physics step and can be use to implement some logic or send data to another interface (such as PX4 through mavlink or ROS2).
    The methods 'start', 'stop' and 'reset' are callbacks that get called when the simulation is started, stoped and reset as the name implies.
    """

    def __init__(self):
        """Initialize the Backend class
        """
        pass

    def update_sensor(self, sensor_type: str, data):
        """Method that when implemented, should handle the receival of sensor data

        Args:
            sensor_type (str): A name that describes the type of sensor
            data (dict): A dictionary that contains the data produced by the sensor
        """
        pass

    def update_state(self, state):
        """Method that when implemented, should handle the receival of the state of the vehicle using this callback

        Args:
            state (State): The current state of the vehicle.
        """
        pass

    def input_reference(self):
        """Method that when implemented, should return a list of desired angular velocities to apply to the vehicle rotors
        """
        return []

    def update(self, dt: float):
        """Method that when implemented, should be used to update the state of the backend and the information being sent/received
        from the communication interface. This method will be called by the simulation on every physics step

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
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

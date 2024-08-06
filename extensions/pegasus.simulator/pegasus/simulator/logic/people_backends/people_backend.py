"""
| File: people_backend.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description:
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
"""
__all__ = ["PeopleBackend"]

class PeopleBackend:
    """
    This class defines the templates for the communication backend. Every person can have at least one backend
    at the same time. Every timestep, the methods 'update_state' and 'update_sensor' are called to update the data produced
    by the simulation, i.e. for every time step the backend will receive teh current state of the person and its sensors. 
    Additionally, the backend must provide a method named 'input_reference' which will be used by the person simulation
    to know the desired angular velocities to apply to the rotors of the person. The method 'update' is called on every
    physics step.
    The methods 'start', 'stop' and 'reset' are callbacks that get called when the simulation is started, stoped and reset as the name implies.
    """

    def __init__(self):
        """Initialize the Backend class
        """
        self._person = None

    """
     Properties
    """
    @property
    def person(self):
        """A reference to the person associated with this backend.

        Returns:
            person: A reference to the vehicle associated with this backend.
        """
        return self._person

    def initialize(self, person):
        """A method that can be invoked when the simulation is starting to give access to the control backend 
        to the entire person object. Even though we provide update_sensor and update_state callbacks that are called
        at every physics step with the latest person state and its sensor data, having access to the full person
        object may prove usefull under some circumstances. This is nice to give users the possibility of overiding
        default person behaviour via this control backend structure.

        Args:
            person (person): A reference to the person that this sensor is associated with
        """
        self._person = person

    def update(self, state, dt: float):
        """Method that when implemented, should be used to update the state of the backend and the information being sent/received
        from the communication interface. This method will be called by the simulation on every physics step

        Args:
            state (State): The current state of the person.
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        pass

    def start(self):
        """Method that when implemented should handle the begining of the simulation of person
        """
        pass

    def stop(self):
        """Method that when implemented should handle the stopping of the simulation of person
        """
        pass

    def reset(self):
        """Method that when implemented, should handle the reset of the person simulation to its original state
        """
        pass

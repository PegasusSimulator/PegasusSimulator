"""
| File: person_controller.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: Base controller class that should be inherited to create a custom controller behaviour for a person in the simulation.
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
"""

class PersonController:

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
            person: A reference to the person associated with this backend.
        """
        return self._person

    def initialize(self, person):
        """A method that can be invoked when the simulation is starting to give access to the person control backend 
        to the entire person object.

        Args:
            person (person): A reference to the person that this sensor is associated with
        """
        self._person = person

    def update_state(self, state):
        """Method that when implemented, should handle the receival of the state of the person using this callback

        Args:
            state (State): The current state of the person.
        """
        pass

    def update(self, dt: float):
        """Method that when implemented, should be used to update the state of the backend and the information being sent/received
        from the communication interface. This method will be called by the simulation on every physics step

        Args:
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

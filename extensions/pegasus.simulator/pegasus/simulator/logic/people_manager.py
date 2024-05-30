"""
| File: people_manager.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the PeopleManager class - a singleton used to manage the people that are spawned in the simulation world
"""

__all__ = ["PeopleManager"]

import carb
from threading import Lock


class PeopleManager:
    """The PeopleManager class is implemented following a singleton pattern. This means that once a person is spawned
    on the world or an instance of the PeopleManager is created, no either will be running at the same time.

    This class keeps track of all the people that are spawned in the simulation world, either trough the extension UI
    or via Python script. Every time a new person object is created, the 'add_person' method is invoked. Additionally, 
    a person is removed, i.e. 'remove_person' gets invoked, every time the '__del__' function of the "Person" object
    gets invoked.
    """

    # The object instance of the people Manager
    _instance = None
    _is_initialized = False
    
    # A dictionary of people that are spawned in the simulator
    _people = {}

    # Lock for safe multi-threading
    _lock: Lock = Lock()

    def __init__(self):
        """
        Constructor for the people manager class.
        """
        pass

    """
    Properties
    """

    @property
    def people(self):
        """
        Returns:
            (list) List of people that were spawned.
        """
        return PeopleManager._people

    """
    Operations
    """

    @staticmethod
    def get_people_manager():
        """
        Method that returns the current people manager.
        """
        return PeopleManager()

    def add_person(self, stage_prefix: str, person):
        """
        Method that adds the people to the person manager.

        Args:
            stage_prefix (str): A string with the name that the person is spawned in the simulator
            person (Person): The person object being added to the person manager.
        """
        PeopleManager._people[stage_prefix] = person

    def get_person(self, stage_prefix: str):
        """Method that returns the person object given its stage prefix. Returns None if there is no person
        associated with that stage prefix

        Args:
            stage_prefix (str): A string with the name that the person is spawned in the simulator

        Returns:
            Person: The person object associated with the stage_prefix
        """
        return PeopleManager._people.get(stage_prefix, None)

    def remove_person(self, stage_prefix: str):
        """
        Method that deletes a person from the person manager.

        Args:
            stage_prefix (str): A string with the name that the person is spawned in the simulator.
        """
        try:
            PeopleManager._people.pop(stage_prefix)
        except:
            pass

    def remove_all_people(self):
        """
        Method that will delete all the people that were spawned from the people manager.
        """

        PeopleManager._people.clear()

    def __new__(cls):
        """Method that allocated memory for a new people_manager. Since the PeopleManager follows a singleton pattern,
        only one instance of PeopleManger object can be in memory at any time.

        Returns:
            PeopleManger: the single instance of the PeopleManager class.
        """

        # Use a lock in here to make sure we do not have a race condition
        # when using multi-threading and creating the first instance of the PeopleManager
        with cls._lock:
            if cls._instance is None:
                cls._instance = object.__new__(cls)
            else:
                carb.log_info("People Manager is defined already, returning the previously defined one")

            return PeopleManager._instance

    def __del__(self):
        """Destructor for the object"""
        PeopleManager._instance = None
        return

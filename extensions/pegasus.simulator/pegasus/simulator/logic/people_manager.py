"""
| File: people_manager.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the PeopleManager class - a singleton used to manage the people that are spawned in the simulation world
"""

__all__ = ["PeopleManager"]

import carb
from threading import Lock

import omni.kit.commands

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

        # Rebuild the navigation mesh using the standard settings
        self.rebuild_nav_mesh()

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

    def rebuild_nav_mesh(height=1.5, radius=0.5, auto_rebake_on_changes=False, auto_rebake_delay_seconds=4, exclude_rigid_bodies=True, view_nav_mesh=False, dynamic_avoidance_enabled=False, navmesh_enabled=False):
        """
        Rebuild the navmesh with the correct settings. Used for the people to move around.
        Called only when the sim with people is requested.
        """

        # Set the size of the people moving in the world
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/exts/omni.anim.navigation.core/navMesh/config/agentHeight',
            value=height)
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/exts/omni.anim.navigation.core/navMesh/config/agentRadius',
            value=radius)
        # Do not rebake the navigation mesh automatically
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/persistent/exts/omni.anim.navigation.core/navMesh/autoRebakeOnChanges',
            value=auto_rebake_on_changes)
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/persistent/exts/omni.anim.navigation.core/navMesh/autoRebakeDelaySeconds',
            value=auto_rebake_delay_seconds)
        # Exclude rigid bodies in the world, such as the drone
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/exts/omni.anim.navigation.core/navMesh/config/excludeRigidBodies',
            value=exclude_rigid_bodies)
        # Do not show the navigation mesh
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/persistent/exts/omni.anim.navigation.core/navMesh/viewNavMesh',
            value=view_nav_mesh)
        print('Navigation mesh rebuilt.')

        # Setup for obstacle avoidance
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled',
            value=dynamic_avoidance_enabled)
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/exts/omni.anim.people/navigation_settings/navmesh_enabled',
            value=navmesh_enabled)

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

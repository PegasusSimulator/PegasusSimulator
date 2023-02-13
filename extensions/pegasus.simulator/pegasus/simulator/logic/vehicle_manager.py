"""
| File: vehicle_manager.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Definition of the VehicleManager class - a singleton used to manage the vehiles that are spawned in the simulation world
"""

__all__ = ["VehicleManager"]

import carb
from threading import Lock


class VehicleManager:
    """The VehicleManager class is implemented following a singleton pattern. This means that once a vehicle is spawned
    on the world or an instance of the VehicleManager is created, no either will be running at the same time.

    This class keeps track of all the vehicles that are spawned in the simulation world, either trough the extension UI
    or via Python script. Every time a new vehicle object is created, the 'add_vehicle' method is invoked. Additionally, 
    a vehicle is removed, i.e. 'remove_vehicle' gets invoked, every time the '__del__' function of the "Vehicle" object
    gets invoked.
    """

    # The object instance of the Vehicle Manager
    _instance = None
    _is_initialized = False
    
    # A dictionary of vehicles that are spawned in the simulator
    _vehicles = {}

    # Lock for safe multi-threading
    _lock: Lock = Lock()

    def __init__(self):
        """
        Constructor for the vehicle manager class.
        """
        pass

    """
    Properties
    """

    @property
    def vehicles(self):
        """
        Returns:
            (list) List of vehicles that were spawned.
        """
        return VehicleManager._vehicles

    """
    Operations
    """

    @staticmethod
    def get_vehicle_manager():
        """
        Method that returns the current vehicle manager.
        """
        return VehicleManager()

    def add_vehicle(self, stage_prefix: str, vehicle):
        """
        Method that adds the vehicles to the vehicle manager.

        Args:
            stage_prefix (str): A string with the name that the vehicle is spawned in the simulator
            vehicle (Vehicle): The vehicle object being added to the vehicle manager.
        """
        VehicleManager._vehicles[stage_prefix] = vehicle

    def get_vehicle(self, stage_prefix: str):
        """Method that returns the vehicle object given its stage prefix. Returns None if there is no vehicle
        associated with that stage prefix

        Args:
            stage_prefix (str): A string with the name that the vehicle is spawned in the simulator

        Returns:
            Vehicle: The vehicle object associated with the stage_prefix
        """
        return VehicleManager._vehicles.get(stage_prefix, None)

    def remove_vehicle(self, stage_prefix: str):
        """
        Method that deletes a vehicle from the vehicle manager.

        Args:
            stage_prefix (str): A string with the name that the vehicle is spawned in the simulator.
        """
        try:
            VehicleManager._vehicles.pop(stage_prefix)
        except:
            pass

    def remove_all_vehicles(self):
        """
        Method that will delete all the vehicles that were spawned from the vehicle manager.
        """

        VehicleManager._vehicles.clear()

    def __new__(cls):
        """Method that allocated memory for a new vehicle_manager. Since the VehicleManager follows a singleton pattern,
        only one instance of VehicleManger object can be in memory at any time.

        Returns:
            VehicleManger: the single instance of the VehicleManager class.
        """

        # Use a lock in here to make sure we do not have a race condition
        # when using multi-threading and creating the first instance of the VehicleManager
        with cls._lock:
            if cls._instance is None:
                cls._instance = object.__new__(cls)
            else:
                carb.log_info("Vehicle Manager is defined already, returning the previously defined one")

            return VehicleManager._instance

    def __del__(self):
        """Destructor for the object"""
        VehicleManager._instance = None
        return

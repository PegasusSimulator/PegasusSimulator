#!/usr/bin/env python
import carb
from threading import Lock

class VehicleManager:

    # The object instance of the Vehicle Manager
    _instance = None

    # Lock for safe multi-threading
    _lock: Lock = Lock()
    
    def __init__(self):
        """
        Constructor for the vehicle manager class
        """

        # A dictionary of vehicles that are spawned in the simulator
        self._vehicles = {}

    """
    Properties
    """

    @property
    def vehicles(self):
        """
        Returns the list of vehicles that were spawned
        """
        return self._vehicles

    """
    Operations
    """

    @staticmethod
    def get_vehicle_manager():
        """
        Method that returns the current vehicle manager
        """
        return VehicleManager()

    def add_vehicle(self, stage_prefix, vehicle):
        """
        Method that adds the vehicles to the vehicle manager
        """
        self._vehicles[stage_prefix] = vehicle

    def remove_vehicle(self, stage_prefix):
        """
        Method that deletes a vehicle from the vehicle manager
        """
        try:
            self._vehicles.pop(stage_prefix) 
        except:
            pass

    def remove_all_vehicles(self):
        """
        Method that will delete all the vehicles that were spawned from the vehicle manager
        """

        
        self._vehicles.clear()

    def __new__(cls):
        """[summary]

        Returns:
            VehicleManger: the single instance of the VehicleManager class 
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
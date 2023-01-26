#!/usr/bin/env python

# Importing Lock in ordef to have a multithread safe Pegasus singleton that manages the entire Pegasus extension
import os
from threading import Lock

# NVidia API imports
import carb
from omni.isaac.core.world import World
from omni.isaac.core.prims import XFormPrim 
import omni.isaac.core.utils.nucleus as nucleus

# Pegasus Simulator internal API
from pegasus_isaac.params import DEFAULT_WORLD_SETTINGS
from pegasus_isaac.logic.vehicles import VehicleManager

class PegasusSimulator:

    # The object instance of the Vehicle Manager
    _instance = None
    _is_initialized = False

    # Lock for safe multi-threading
    _lock: Lock = Lock()

    def __init__(self):

        # If we already have an instance of the PegasusSimulator, do not overwrite it!
        if PegasusSimulator._is_initialized:
            return

        carb.log_warn("Initializing the Pegasus Simulator Extension")

        # Get a handle to the vehicle manager instance which will manage which vehicles are spawned in the world
        # to be controlled and simulated
        self._vehicle_manager = VehicleManager()

        # Initialize the world with the default simulation settings
        self._world_settings = DEFAULT_WORLD_SETTINGS
        self._world = World(**self._world_settings)

    @property
    def world(self):
        return self._world

    def clear_world(self):
        self._world.clear()

    def load_nvidia_environment(self, environment_asset: str="/Hospital/hospital.usd"):
        """
        Method that is used to load NVidia internally provided USD stages into the simulaton World 
        """
        
        # Get the nvidia assets root path
        nvidia_assets_path = nucleus.get_assets_root_path()

        # Define the environments path inside the NVidia assets
        environments_path = "/Isaac/Environments"

        # Get the complete usd path
        usd_path = nvidia_assets_path + environments_path + environment_asset

        # Try to load the asset into the world
        self.load_asset(usd_path, "/World/layout")

    def load_asset(self, usd_asset: str, stage_prefix: str):
        """
        Method that will attempt to load an asset into the current simulation world, given the USD asset path
        """
        
        # Try to check if there is already a prim with the same stage prefix in the stage
        if self._world.stage.GetPrimAtPath(stage_prefix):
            raise Exception("A primitive already exists at the specified path")
        
        # Create the stage primitive and load the usd into it
        prim = self._world.stage.DefinePrim(stage_prefix)
        success = prim.GetReferences().AddReference(usd_asset)

        if not success:
            raise Exception("The usd asset" + usd_asset + "is not load at stage path " + stage_prefix)
        
        # Wrap the primitive with and XFormPrim and add it to the scene
        #x_form_prim = XFormPrim(prim)
        #self._world.scene.add(x_form_prim)

    def __new__(cls):
        """[summary]

        Returns:
            VehicleManger: the single instance of the VehicleManager class 
        """

        # Use a lock in here to make sure we do not have a race condition
        # when using multi-threading and creating the first instance of the Pegasus extension manager
        with cls._lock:
            if cls._instance is None:
                cls._instance = object.__new__(cls)
        
        return PegasusSimulator._instance

    def __del__(self):
        """Destructor for the object"""
        PegasusSimulator._instance = None
        return
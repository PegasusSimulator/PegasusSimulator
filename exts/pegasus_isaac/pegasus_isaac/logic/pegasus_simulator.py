#!/usr/bin/env python

# Importing Lock in ordef to have a multithread safe Pegasus singleton that manages the entire Pegasus extension
import gc
import asyncio
from threading import Lock

# NVidia API imports
import carb
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import clear_stage
from omni.isaac.core.utils.viewports import set_camera_view
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
        PegasusSimulator._is_initialized = True

        # Get a handle to the vehicle manager instance which will manage which vehicles are spawned in the world
        # to be controlled and simulated
        self._vehicle_manager = VehicleManager()

        # Initialize the world with the default simulation settings
        self._world_settings = DEFAULT_WORLD_SETTINGS
        self._world = World(**self._world_settings)
        asyncio.ensure_future(self._world.initialize_simulation_context_async())

    @property
    def world(self):
        return self._world

    @property
    def vehicle_manager(self):
        return self._vehicle_manager

    def get_vehicle(self, stage_prefix:str):
        return self._vehicle_manager.vehicles[stage_prefix]

    def get_all_vehicles(self):
        """
        Method that returns a list of vehicles that are considered active in the simulator
        """
        return self._vehicle_manager.vehicles

    def clear_scene(self):
        """
        Method that when invoked will clear all vehicles and the simulation environment, leaving only an empty
        world with a physics environment
        """

        # If the physics simulation was running, stop it first
        if self.world is not None:
            self.world.stop()

        # Clear the world
        if self.world is not None:
            self.world.clear_all_callbacks()
            self.world.clear()

        # Clear the stage
        clear_stage()

        # Remove all the robots that were spawned
        self._vehicle_manager.remove_all_vehicles()

        # Call python's garbage collection
        gc.collect()

        # Re-initialize the physics context
        asyncio.ensure_future(self._world.initialize_simulation_context_async())
        carb.log_info("Current scene and its vehicles has been deleted")
    
    async def load_environment_async(self, usd_path: str):

        # Reset and pause the world simulation
        await self.world.reset_async()
        await self.world.stop_async()     

        # Load the USD asset that will be used for the environment
        try:
            self.load_asset(usd_path, "/World/layout")
        except Exception as e:
            carb.log_warn("Could not load the desired environment: " + str(e))

        carb.log_info("A new environment has been loaded successfully")
    
    def load_environment(self, usd_path: str):
        asyncio.ensure_future(self.load_environment_async(usd_path))
    
    def load_nvidia_environment(self, environment_asset: str="Hospital/hospital.usd"):
        """
        Method that is used to load NVidia internally provided USD stages into the simulaton World 
        """
        
        # Get the nvidia assets root path
        nvidia_assets_path = nucleus.get_assets_root_path()

        # Define the environments path inside the NVidia assets
        environments_path = "/Isaac/Environments"

        # Get the complete usd path
        usd_path = nvidia_assets_path + environments_path + '/' + environment_asset

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

    def set_viewport_camera(self, camera_position, camera_target):
        # Set the camera view to a fixed value
        set_camera_view(eye=camera_position, target=camera_target)

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
        PegasusSimulator._is_initialized = False
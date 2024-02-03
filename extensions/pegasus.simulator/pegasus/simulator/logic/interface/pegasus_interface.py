"""
| File: pegasus_interface.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Definition of the PegasusInterface class (a singleton) that is used to manage the Pegasus framework.
"""

__all__ = ["PegasusInterface"]

# Importing Lock in ordef to have a multithread safe Pegasus singleton that manages the entire Pegasus extension
import gc
import yaml
import asyncio
import os
from threading import Lock

# NVidia API imports
import carb
import omni.kit.app
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import clear_stage, create_new_stage_async, update_stage_async
from omni.isaac.core.utils.viewports import set_camera_view
import omni.isaac.core.utils.nucleus as nucleus

# Pegasus Simulator internal API
from pegasus.simulator.params import DEFAULT_WORLD_SETTINGS, SIMULATION_ENVIRONMENTS, CONFIG_FILE
from pegasus.simulator.logic.vehicle_manager import VehicleManager


class PegasusInterface:
    """
    PegasusInterface is a singleton class (there is only one object instance at any given time) that will be used
    to 
    """

    # The object instance of the Vehicle Manager
    _instance = None
    _is_initialized = False

    # Lock for safe multi-threading
    _lock: Lock = Lock()

    def __init__(self):
        """
        Initialize the PegasusInterface singleton object (only runs once at a time)
        """

        # If we already have an instance of the PegasusInterface, do not overwrite it!
        if PegasusInterface._is_initialized:
            return

        carb.log_info("Initializing the Pegasus Simulator Extension")
        PegasusInterface._is_initialized = True

        # Get a handle to the vehicle manager instance which will manage which vehicles are spawned in the world
        # to be controlled and simulated
        self._vehicle_manager = VehicleManager()

        # Initialize the world with the default simulation settings
        self._world_settings = DEFAULT_WORLD_SETTINGS
        self._world = None
        #self.initialize_world()

        # Initialize the latitude, longitude and altitude of the simulated environment at the (0.0, 0.0, 0.0) coordinate
        # from the extension configuration file
        self._latitude, self._longitude, self._altitude = self._get_global_coordinates_from_config()

        # Get the px4_path from the extension configuration file
        self._px4_path: str = self._get_px4_path_from_config()
        self._px4_default_airframe: str = self._get_px4_default_airframe_from_config()
        carb.log_info("Default PX4 path:" + str(self._px4_path))

    @property
    def world(self):
        """The current omni.isaac.core.world World instance

        Returns:
            omni.isaac.core.world: The world instance
        """
        return self._world

    @property
    def vehicle_manager(self):
        """The instance of the VehicleManager.

        Returns:
            VehicleManager: The current instance of the VehicleManager.
        """
        return self._vehicle_manager
    
    @property
    def latitude(self):
        """The latitude of the origin of the simulated world in degrees.

        Returns:
            float: The latitude of the origin of the simulated world in degrees.
        """
        return self._latitude
    
    @property
    def longitude(self):
        """The longitude of the origin of the simulated world in degrees.

        Returns:
            float: The longitude of the origin of the simulated world in degrees.
        """
        return self._longitude
    
    @property
    def altitude(self):
        """The altitude of the origin of the simulated world in meters.

        Returns:
            float: The latitude of the origin of the simulated world in meters.
        """
        return self._altitude
    
    @property
    def px4_path(self):
        """A string with the installation directory for PX4 (if it was setup). Otherwise it is None.

        Returns:
            str: A string with the installation directory for PX4 (if it was setup). Otherwise it is None.
        """
        return self._px4_path
    
    @property
    def px4_default_airframe(self):
        """A string with the PX4 default airframe (if it was setup). Otherwise it is None.

        Returns:
            str: A string with the PX4 default airframe (if it was setup). Otherwise it is None.
        """
        return self._px4_default_airframe
    
    def set_global_coordinates(self, latitude=None, longitude=None, altitude=None):
        """Method that can be used to set the latitude, longitude and altitude of the simulation world at the origin.

        Args:
            latitude (float): The latitude of the origin of the simulated world in degrees. Defaults to None.
            longitude (float): The longitude of the origin of the simulated world in degrees. Defaults to None.
            altitude (float): The altitude of the origin of the simulated world in meters. Defaults to None.
        """

        if latitude is not None:
            self._latitude = latitude

        if longitude is not None:
            self._longitude = longitude

        if self.altitude is not None:
            self._altitude = altitude

        carb.log_warn("New global coordinates set to: " + str(self._latitude) + ", " + str(self._longitude) + ", " + str(self._altitude))

    def initialize_world(self):
        """Method that initializes the world object
        """
        self._world = World(**self._world_settings)
        #asyncio.ensure_future(self._world.initialize_simulation_context_async())

    def get_vehicle(self, stage_prefix: str):
        """Method that returns the vehicle object given its 'stage_prefix', i.e., the name the vehicle was spawned with in the simulator.

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. 

        Returns:
            Vehicle: Returns a vehicle object that was spawned with the given 'stage_prefix'
        """
        return self._vehicle_manager.vehicles[stage_prefix]

    def get_all_vehicles(self):
        """
        Method that returns a list of vehicles that are considered active in the simulator

        Returns:
            list: A list of all vehicles that are currently instantiated.
        """
        return self._vehicle_manager.vehicles

    def get_default_environments(self):
        """
        Method that returns a dictionary containing all the default simulation environments and their path
        """
        return SIMULATION_ENVIRONMENTS

    def generate_quadrotor_config_from_yaml(self, file: str):
        """_summary_

        Args:
            file (str): _description_

        Returns:
            _type_: _description_
        """

        # Load the quadrotor configuration data from the given yaml file
        with open(file) as f:
            data = yaml.safe_load(f)

        return self.generate_quadrotor_config_from_dict(data)

    def clear_scene(self):
        """
        Method that when invoked will clear all vehicles and the simulation environment, leaving only an empty world with a physics environment.
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

    async def load_environment_async(self, usd_path: str, force_clear: bool=False):
        """Method that loads a given world (specified in the usd_path) into the simulator asynchronously.

        Args:
            usd_path (str): The path where the USD file describing the world is located.
            force_clear (bool): Whether to perform a clear before loading the asset. Defaults to False. 
            It should be set to True only if the method is invoked from an App (GUI mode).
        """

        # Reset and pause the world simulation (only if force_clear is true)
        # This is done to maximize the support between running in GUI as extension vs App
        if force_clear == True:

            # Create a new stage and initialize (or re-initialized) the world
            await create_new_stage_async()
            self._world = World(**self._world_settings)
            await self._world.initialize_simulation_context_async()
            self._world = World.instance()

            await self.world.reset_async()
            await self.world.stop_async()

        # Load the USD asset that will be used for the environment
        try:
            self.load_asset(usd_path, "/World/layout")
        except Exception as e:
            carb.log_warn("Could not load the desired environment: " + str(e))

        carb.log_info("A new environment has been loaded successfully")

    def load_environment(self, usd_path: str, force_clear: bool=False):
        """Method that loads a given world (specified in the usd_path) into the simulator. If invoked from a python app,
        this method should have force_clear=False, as the world reset and stop are performed asynchronously by this method, 
        and when we are operating in App mode, we want everything to run in sync.

        Args:
            usd_path (str): The path where the USD file describing the world is located.
            force_clear (bool): Whether to perform a clear before loading the asset. Defaults to False.
        """
        asyncio.ensure_future(self.load_environment_async(usd_path, force_clear))

    def load_nvidia_environment(self, environment_asset: str = "Hospital/hospital.usd"):
        """
        Method that is used to load NVidia internally provided USD stages into the simulaton World

        Args:
            environment_asset (str): The name of the nvidia asset inside the /Isaac/Environments folder. Default to Hospital/hospital.usd.
        """

        # Get the nvidia assets root path
        nvidia_assets_path = nucleus.get_assets_root_path()

        # Define the environments path inside the NVidia assets
        environments_path = "/Isaac/Environments"

        # Get the complete usd path
        usd_path = nvidia_assets_path + environments_path + "/" + environment_asset

        # Try to load the asset into the world
        self.load_asset(usd_path, "/World/layout")

    def load_asset(self, usd_asset: str, stage_prefix: str):
        """
        Method that will attempt to load an asset into the current simulation world, given the USD asset path.

        Args:
            usd_asset (str): The path where the USD file describing the world is located.
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. 
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
        """Sets the viewport camera to given position and makes it point to another target position.

        Args:
            camera_position (list): A list with [X, Y, Z] coordinates of the camera in ENU inertial frame.
            camera_target (list): A list with [X, Y, Z] coordinates of the target that the camera should point to in the ENU inertial frame.
        """
        # Set the camera view to a fixed value
        set_camera_view(eye=camera_position, target=camera_target)

    def set_world_settings(self, physics_dt=None, stage_units_in_meters=None, rendering_dt=None):
        """
        Set the current world settings to the pre-defined settings. TODO - finish the implementation of this method.
        For now these new setting will never override the default ones.
        """

        # Set the physics engine update rate
        if physics_dt is not None:
            self._world_settings["physics_dt"] = physics_dt

        # Set the units of the simulator to meters
        if stage_units_in_meters is not None:
            self._world_settings["stage_units_in_meters"] = stage_units_in_meters

        # Set the render engine update rate (might not be the same as the physics engine)
        if rendering_dt is not None:
            self._world_settings["rendering_dt"] = rendering_dt

    def _get_px4_path_from_config(self):
        """
        Method that reads the configured PX4 installation directory from the extension configuration file 

        Returns:
            str: A string with the path to the px4 configuration directory or empty string ''
        """

        px4_dir = ""
        
        # Open the configuration file. If it fails, just return the empty path
        try:
            with open(CONFIG_FILE, 'r') as f:
                data = yaml.safe_load(f)
            px4_dir = os.path.expanduser(data.get("px4_dir", None))
        except:
            carb.log_warn("Could not retrieve px4_dir from: " + str(CONFIG_FILE))

        return px4_dir
    
    def _get_px4_default_airframe_from_config(self):
        """
        Method that reads the configured PX4 default airframe from the extension configuration file 

        Returns:
            str: A string with the path to the PX4 default airframe or empty string ''
        """
        px4_default_airframe = ""
        
        # Open the configuration file. If it fails, just return the empty path
        try:
            with open(CONFIG_FILE, 'r') as f:
                data = yaml.safe_load(f)
            px4_default_airframe = os.path.expanduser(data.get("px4_default_airframe", None))
        except:
            carb.log_warn("Could not retrieve px4_default_airframe from: " + str(CONFIG_FILE))

        return px4_default_airframe


    def _get_global_coordinates_from_config(self):
        """Method that reads the default latitude, longitude and altitude from the extension configuration file

        Returns:
            (float, float, float): A tuple of 3 floats with the latitude, longitude and altitude to use as the origin of the world
        """

        latitude = 0.0
        longitude = 0.0
        altitude = 0.0

        # Open the configuration file. If it fails, just return the empty path
        try:
            with open(CONFIG_FILE, 'r') as f:
                data = yaml.safe_load(f)
                
                # Try to read the coordinates from the configuration file
                global_coordinates = data.get("global_coordinates", {})
                latitude = global_coordinates.get("latitude", 0.0)
                longitude = global_coordinates.get("longitude", 0.0)
                altitude = global_coordinates.get("altitude", 0.0)
        except:
            carb.log_warn("Could not retrieve the global coordinates from: " + str(CONFIG_FILE))

        return (latitude, longitude, altitude)

    def set_px4_path(self, path: str):
        """Method that allows a user to save a new px4 directory in the configuration files of the extension.

        Args:
            absolute_path (str): The new path of the px4-autopilot installation directory
        """
        
        # Save the new path for current use during this simulation
        self._px4_path = os.path.expanduser(path)

        # Save the new path in the configurations file for the next simulations
        try:

            # Open the configuration file and the all the configurations that it contains
            with open(CONFIG_FILE, 'r') as f:
                data = yaml.safe_load(f)

            # Open the configuration file. If it fails, just warn in the console
            with open(CONFIG_FILE, 'w') as f:
                data["px4_dir"] = path
                yaml.dump(data, f)
        except:
            carb.log_warn("Could not save px4_dir to: " + str(CONFIG_FILE))

        carb.log_warn("New px4_dir set to: " + str(self._px4_path))

    def set_px4_default_airframe(self, airframe: str):
        """Method that allows a user to save a new px4 default airframe for the extension.

        Args:
            absolute_path (str): The new px4 default airframe
        """
        
        # Save the new path for current use during this simulation
        self._px4_default_airframe = airframe

        # Save the new path in the configurations file for the next simulations
        try:

            # Open the configuration file and the all the configurations that it contains
            with open(CONFIG_FILE, 'r') as f:
                data = yaml.safe_load(f)

            # Open the configuration file. If it fails, just warn in the console
            with open(CONFIG_FILE, 'w') as f:
                data["px4_default_airframe"] = airframe
                yaml.dump(data, f)
        except:
            carb.log_warn("Could not save px4_default_airframe to: " + str(CONFIG_FILE))

        carb.log_warn("New px4_default_airframe set to: " + str(self._px4_default_airframe))

    def set_default_global_coordinates(self):
        """
        Method that sets the latitude, longitude and altitude from the pegasus interface to the 
        default global coordinates specified in the extension configuration file
        """
        self._latitude, self._longitude, self._altitude = self._get_global_coordinates_from_config()

    def set_new_default_global_coordinates(self, latitude: float=None, longitude: float=None, altitude: float=None):
        
        # Set the current global coordinates to the new default global coordinates
        self.set_global_coordinates(latitude, longitude, altitude)

        # Update the default global coordinates in the configuration file
        try:
            # Open the configuration file and the all the configurations that it contains
            with open(CONFIG_FILE, 'r') as f:
                data = yaml.safe_load(f)

            # Open the configuration file. If it fails, just warn in the console
            with open(CONFIG_FILE, 'w') as f:

                if latitude is not None:
                    data["global_coordinates"]["latitude"] = latitude

                if longitude is not None:
                    data["global_coordinates"]["longitude"] = longitude

                if altitude is not None:
                    data["global_coordinates"]["altitude"] = altitude
                
                # Save the updated configurations    
                yaml.dump(data, f)
        except:
            carb.log_warn("Could not save the new global coordinates to: " + str(CONFIG_FILE))

        carb.log_warn("New global coordinates set to: latitude=" + str(latitude) + ", longitude=" + str(longitude) + ", altitude=" + str(altitude))


    def __new__(cls):
        """Allocates the memory and creates the actual PegasusInterface object is not instance exists yet. Otherwise,
        returns the existing instance of the PegasusInterface class.

        Returns:
            VehicleManger: the single instance of the VehicleManager class
        """

        # Use a lock in here to make sure we do not have a race condition
        # when using multi-threading and creating the first instance of the Pegasus extension manager
        with cls._lock:
            if cls._instance is None:
                cls._instance = object.__new__(cls)

        return PegasusInterface._instance

    def __del__(self):
        """Destructor for the object. Destroys the only existing instance of this class."""
        PegasusInterface._instance = None
        PegasusInterface._is_initialized = False
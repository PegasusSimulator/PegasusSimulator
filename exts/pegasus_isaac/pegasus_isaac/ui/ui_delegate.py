#!/usr/bin/env python

# Python libraries for multithreading and garbage collection
import gc
import asyncio

# Omniverse extensions
import carb
import omni.ui as ui
from omni.isaac.core import World
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.stage import create_new_stage, set_stage_up_axis, clear_stage, add_reference_to_stage, get_current_stage

# Extension Configurations
from pegasus_isaac.params import ROBOTS, SIMULATION_ENVIRONMENTS

# Vehicle Manager to spawn Vehicles
from pegasus_isaac.logic.vehicles.quadrotor import Quadrotor
from pegasus_isaac.logic.vehicles.vehicle_manager import VehicleManager

class UIDelegate:
    """
    Object that will interface between the logic/dynamic simulation part of the extension and the Widget UI
    """

    def __init__(self, world: World, world_settings):
        
        self._world = world
        self._world_settings = world_settings

        # Attribute that holds the currently selected scene from the dropdown menu
        self._scene_dropdown: ui.AbstractItemModel = None
        self._scene_names = list(SIMULATION_ENVIRONMENTS.keys())
        
        # Attribute that hold the currently selected vehicle from the dropdown menu
        self._vehicle_dropdown: ui.AbstractItemModel = None
        self._vehicles_names = list(ROBOTS.keys())

        # Get an instance of the vehicle manager
        self._vehicle_manager = VehicleManager()

        # Selected option for broadcasting the simulated vehicle (PX4+ROS2 or just ROS2)
        # By default we assume PX4
        self._streaming_backend: str = "px4"

    def set_scene_dropdown(self, scene_dropdown_model: ui.AbstractItemModel):
        self._scene_dropdown = scene_dropdown_model

    def set_vehicle_dropdown(self, vehicle_dropdown_model: ui.AbstractItemModel):
        self._vehicle_dropdown = vehicle_dropdown_model

    def set_streaming_backend(self, backend: str="px4"):
        carb.log_info("Chosen option: " + backend)
        self._streaming_backend = backend
    
    """
    ---------------------------------------------------------------------
    Callbacks to handle user interaction with the extension widget window
    ---------------------------------------------------------------------
    """

    def on_load_scene(self):
        """
        Method that should be invoked when the button to load the selected world is pressed
        """

        # Asynchronous auxiliary function in order for the UI to be responsive while loading the world
        async def load_world_async(self):

            # Setup a new world (even if one already existed)
            self._world = World(**self._world_settings)
            await self._world.initialize_simulation_context_async()

            # Reset and pause the world simulation
            await self._world.reset_async()
            await self._world.stop_async()

            # Load a ground in the world
            # TODO - add the loading of the custom scene here
            self._world.scene.add_default_ground_plane()

        # --------------------------
        # Function logic starts here
        # -------------------------- 

        carb.log_warn("New scene has been loaded")

        # Check if a scene is selected in the drop-down menu
        if self._scene_dropdown is not None:

            # Get the id of the selected environment from the list
            environemnt_index = self._scene_dropdown.get_item_value_model().as_int

            # Create a new empty with the correct settings and initialize it
            asyncio.ensure_future(load_world_async(self))


    def on_clear_scene(self):
        """
        Method that should be invoked when the clear world button is pressed
        """
        carb.log_warn("Current scene and its vehicles has been deleted")

        # If the physics simulation was running, stop it first
        if self._world is not None:
            self._world.stop()

        # Clear the world
        if self._world is not None:
            self._world.clear_all_callbacks()
            self._world.clear()

        # Clear the stage
        clear_stage()
        
        # Cleanup the world pointer
        self._world = None

        # Remove all the robots that were spawned
        self._vehicle_manager.remove_all_vehicles()

        # Call python's garbage collection
        gc.collect()

    def on_load_vehicle(self):
        """
        Method that should be invoked when the button to load the selected vehicle is pressed
        """
        carb.log_warn("A new vehicle has been loaded")

        # Check fi a vehicle is selected in the drop-down menu
        if self._vehicle_dropdown is not None:

            # Get the id of the selected vehicle from the list
            vehicle_index = self._vehicle_dropdown.get_item_value_model().as_int

            # Get the name of the selected vehicle
            selected_robot = self._vehicles_names[vehicle_index]

            # Try to spawn the selected robot in the world to the specified namespace
            Quadrotor("/World/quadrotor", ROBOTS[selected_robot], self._world)

            carb.log_warn("Spawned the robot: " + selected_robot + "in ")
    
    def on_set_viewport_camera(self):
        """
        Method that should be invoked when the button to set the viewport camera pose is pressed
        """
        carb.log_warn("The viewport camera pose has been adjusted")

        # Set the camera view to a fixed value
        set_camera_view(eye=np.array([5, 5, 5]), target=np.array([0, 0, 0])) 

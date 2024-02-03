"""
| File: ui_delegate.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Definition of the UiDelegate which is an abstraction layer betweeen the extension UI and code logic features
"""

# External packages
import os
import asyncio
from scipy.spatial.transform import Rotation

# Omniverse extensions
import carb
import omni.ui as ui

# Extension Configurations
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Vehicle Manager to spawn Vehicles
from pegasus.simulator.logic.backends import MavlinkBackend, MavlinkBackendConfig #, ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.vehicle_manager import VehicleManager


class UIDelegate:
    """
    Object that will interface between the logic/dynamic simulation part of the extension and the Widget UI
    """

    def __init__(self):

        # The window that will be bound to this delegate
        self._window = None

        # Get an instance of the pegasus simulator
        self._pegasus_sim: PegasusInterface = PegasusInterface()

        # Attribute that holds the currently selected scene from the dropdown menu
        self._scene_dropdown: ui.AbstractItemModel = None
        self._scene_names = list(SIMULATION_ENVIRONMENTS.keys())

        # Selected latitude, longitude and altitude
        self._latitude_field: ui.AbstractValueModel = None
        self._latitude = PegasusInterface().latitude
        self._longitude_field: ui.AbstractValueModel = None
        self._longitude = PegasusInterface().longitude
        self._altitude_field: ui.AbstractValueModel = None
        self._altitude = PegasusInterface().altitude

        # Attribute that hold the currently selected vehicle from the dropdown menu
        self._vehicle_dropdown: ui.AbstractItemModel = None
        self._vehicles_names = list(ROBOTS.keys())

        # Get an instance of the vehicle manager
        self._vehicle_manager = VehicleManager()

        # Selected option for broadcasting the simulated vehicle (PX4+ROS2 or just ROS2)
        # By default we assume PX4
        self._streaming_backend: str = "px4"

        # Selected value for the the id of the vehicle
        self._vehicle_id_field: ui.AbstractValueModel = None
        self._vehicle_id: int = 0

        # Attribute that will save the model for the px4-autostart checkbox
        self._px4_autostart_checkbox: ui.AbstractValueModel = None
        self._autostart_px4: bool = True

        # Atributes to store the path for the Px4 directory
        self._px4_directory_field: ui.AbstractValueModel = None
        self._px4_dir: str = PegasusInterface().px4_path

        # Atributes to store the PX4 airframe
        self._px4_airframe_field: ui.AbstractValueModel = None
        self._px4_airframe: str = self._pegasus_sim.px4_default_airframe

    def set_window_bind(self, window):
        self._window = window

    def set_scene_dropdown(self, scene_dropdown_model: ui.AbstractItemModel):
        self._scene_dropdown = scene_dropdown_model
    
    def set_latitude_field(self, latitude_model: ui.AbstractValueModel):
        self._latitude_field = latitude_model
    
    def set_longitude_field(self, longitude_model: ui.AbstractValueModel):
        self._longitude_field = longitude_model

    def set_altitude_field(self, altitude_model: ui.AbstractValueModel):
        self._altitude_field = altitude_model

    def set_vehicle_dropdown(self, vehicle_dropdown_model: ui.AbstractItemModel):
        self._vehicle_dropdown = vehicle_dropdown_model

    def set_vehicle_id_field(self, vehicle_id_field: ui.AbstractValueModel):
        self._vehicle_id_field = vehicle_id_field

    def set_streaming_backend(self, backend: str = "px4"):
        carb.log_info("Chosen option: " + backend)
        self._streaming_backend = backend

    def set_px4_autostart_checkbox(self, checkbox_model:ui.AbstractValueModel):
        self._px4_autostart_checkbox = checkbox_model

    def set_px4_directory_field(self, directory_field_model: ui.AbstractValueModel):
        self._px4_directory_field = directory_field_model

    def set_px4_airframe_field(self, airframe_field_model: ui.AbstractValueModel):
        self._px4_airframe_field = airframe_field_model

    """
    ---------------------------------------------------------------------
    Callbacks to handle user interaction with the extension widget window
    ---------------------------------------------------------------------
    """

    def on_load_scene(self):
        """
        Method that should be invoked when the button to load the selected world is pressed
        """

        # Check if a scene is selected in the drop-down menu
        if self._scene_dropdown is not None:

            # Get the id of the selected environment from the list
            environemnt_index = self._scene_dropdown.get_item_value_model().as_int

            # Get the name of the selected world
            selected_world = self._scene_names[environemnt_index]

            # Try to spawn the selected world
            asyncio.ensure_future(self._pegasus_sim.load_environment_async(SIMULATION_ENVIRONMENTS[selected_world], force_clear=True))

    def on_set_new_global_coordinates(self):
        """
        Method that gets invoked to set new global coordinates for this simulation
        """
        self._pegasus_sim.set_global_coordinates(
            self._latitude_field.get_value_as_float(),
            self._longitude_field.get_value_as_float(),
            self._altitude_field.get_value_as_float())
        
    def on_reset_global_coordinates(self):
        """
        Method that gets invoked to set the global coordinates to the defaults saved in the extension configuration file
        """
        self._pegasus_sim.set_default_global_coordinates()

        self._latitude_field.set_value(self._pegasus_sim.latitude)
        self._longitude_field.set_value(self._pegasus_sim.longitude)
        self._altitude_field.set_value(self._pegasus_sim.altitude)

    def on_set_new_default_global_coordinates(self):
        """
        Method that gets invoked to set new defualt global coordinates for this simulation. This will attempt
        to save the current coordinates as new defaults for the extension itself
        """
        self._pegasus_sim.set_new_default_global_coordinates(
            self._latitude_field.get_value_as_float(),
            self._longitude_field.get_value_as_float(),
            self._altitude_field.get_value_as_float()
        )

    def on_clear_scene(self):
        """
        Method that should be invoked when the clear world button is pressed
        """
        self._pegasus_sim.clear_scene()

    def on_load_vehicle(self):
        """
        Method that should be invoked when the button to load the selected vehicle is pressed
        """

        async def async_load_vehicle():

            # Check if we already have a physics environment activated. If not, then activate it
            # and only after spawn the vehicle. This is to avoid trying to spawn a vehicle without a physics
            # environment setup. This way we can even spawn a vehicle in an empty world and it won't care
            if hasattr(self._pegasus_sim.world, "_physics_context") == False:
                await self._pegasus_sim.world.initialize_simulation_context_async()

            # Check if a vehicle is selected in the drop-down menu
            if self._vehicle_dropdown is not None and self._window is not None:

                # Get the id of the selected vehicle from the list
                vehicle_index = self._vehicle_dropdown.get_item_value_model().as_int

                # Get the name of the selected vehicle
                selected_robot = self._vehicles_names[vehicle_index]

                # Get the id of the selected vehicle
                self._vehicle_id = self._vehicle_id_field.get_value_as_int()

                # Get the desired position and orientation of the vehicle from the UI transform
                pos, euler_angles = self._window.get_selected_vehicle_attitude()

                # Read if we should auto-start px4 from the checkbox
                px4_autostart = self._px4_autostart_checkbox.get_value_as_bool()

                # Read the PX4 path from the field
                px4_path = os.path.expanduser(self._px4_directory_field.get_value_as_string())

                # Read the PX4 airframe from the field
                px4_airframe = self._px4_airframe_field.get_value_as_string()

                # Create the multirotor configuration
                mavlink_config = MavlinkBackendConfig({
                    "vehicle_id": self._vehicle_id,
                    "px4_autolaunch": px4_autostart,
                    "px4_dir": px4_path,
                    "px4_vehicle_model": px4_airframe
                })
                config_multirotor = MultirotorConfig()
                config_multirotor.backends = [MavlinkBackend(mavlink_config)]

                #ros2 = ROS2Backend(self._vehicle_id)

                # Try to spawn the selected robot in the world to the specified namespace
                Multirotor(
                    "/World/quadrotor",
                    ROBOTS[selected_robot],
                    self._vehicle_id,
                    pos,
                    Rotation.from_euler("XYZ", euler_angles, degrees=True).as_quat(),
                    config=config_multirotor,
                )

            # Log that a vehicle of the type multirotor was spawned in the world via the extension UI
                carb.log_info("Spawned the robot: " + selected_robot + " using the Pegasus Simulator UI")
            else:
                # Log that it was not possible to spawn the vehicle in the world using the Pegasus Simulator UI
                carb.log_error("Could not spawn the robot using the Pegasus Simulator UI")

        # Run the actual vehicle spawn async so that the UI does not freeze
        asyncio.ensure_future(async_load_vehicle())        

    def on_set_viewport_camera(self):
        """
        Method that should be invoked when the button to set the viewport camera pose is pressed
        """
        carb.log_warn("The viewport camera pose has been adjusted")

        if self._window:

            # Get the current camera position value
            camera_position, camera_target = self._window.get_selected_camera_pos()

            if camera_position is not None and camera_target is not None:

                # Set the camera view to a fixed value
                self._pegasus_sim.set_viewport_camera(eye=camera_position, target=camera_target)
    
    def on_set_new_default_px4_path(self):
        """
        Method that will try to update the new PX4 autopilot path with whatever is passed on the string field
        """
        carb.log_warn("A new default PX4 Path will be set for the extension.")

        # Read the current path from the field
        path = self._px4_directory_field.get_value_as_string()

        # Set the path using the pegasus interface
        self._pegasus_sim.set_px4_path(path)

    def on_reset_px4_path(self):
        """
        Method that will reset the string field to the default PX4 path
        """
        carb.log_warn("Reseting the path to the default one")
        self._px4_directory_field.set_value(self._pegasus_sim.px4_path)

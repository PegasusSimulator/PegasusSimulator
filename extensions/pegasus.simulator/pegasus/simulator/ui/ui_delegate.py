"""
| File: ui_delegate.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Definition of the UiDelegate which is an abstraction layer betweeen the extension UI and code logic features
"""

# External packages
import os
from scipy.spatial.transform import Rotation

# Omniverse extensions
import carb
import omni.ui as ui

# Extension Configurations
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Vehicle Manager to spawn Vehicles
from pegasus.simulator.logic.backends import MavlinkBackend, MavlinkBackendConfig
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
        carb.log_warn(self._px4_dir)

        # Atributes to store the PX4 airframe
        self._px4_airframe_field: ui.AbstractValueModel = None
        self._px4_airframe: str = 'iris'
        carb.log_warn(self._px4_airframe)

    def set_window_bind(self, window):
        self._window = window

    def set_scene_dropdown(self, scene_dropdown_model: ui.AbstractItemModel):
        self._scene_dropdown = scene_dropdown_model

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
            self._pegasus_sim.load_environment(SIMULATION_ENVIRONMENTS[selected_world], force_clear=True)

    def on_clear_scene(self):
        """
        Method that should be invoked when the clear world button is pressed
        """
        self._pegasus_sim.clear_scene()

    def on_load_vehicle(self):
        """
        Method that should be invoked when the button to load the selected vehicle is pressed
        """

        # Check fi a vehicle is selected in the drop-down menu
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

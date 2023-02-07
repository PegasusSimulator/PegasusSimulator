# Copyright (c) 2023, Marcelo Jacinto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# External packages
from scipy.spatial.transform import Rotation

# Omniverse extensions
import carb
import omni.ui as ui

# Extension Configurations
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.pegasus_simulator import PegasusSimulator

# Vehicle Manager to spawn Vehicles
from pegasus.simulator.logic.backends import MavlinkBackend, MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.vehicles.vehicle_manager import VehicleManager


class UIDelegate:
    """
    Object that will interface between the logic/dynamic simulation part of the extension and the Widget UI
    """

    def __init__(self):

        # The window that will be bound to this delegate
        self._window = None

        # Get an instance of the pegasus simulator
        self._pegasus_sim: PegasusSimulator = PegasusSimulator()

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
            self._pegasus_sim.load_environment(SIMULATION_ENVIRONMENTS[selected_world])

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

            # Create the multirotor configuration
            mavlink_config = MavlinkBackendConfig()
            mavlink_config.vehicle_id = self._vehicle_id
            config_multirotor = MultirotorConfig()
            config_multirotor.backends = [MavlinkBackend(mavlink_config)]

            # Try to spawn the selected robot in the world to the specified namespace
            Multirotor(
                "/World/quadrotor",
                ROBOTS[selected_robot],
                self._vehicle_id,
                self._pegasus_sim.world,
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

"""
| File: ui_window.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Definition of WidgetWindow which contains all the UI code that defines the extension GUI
"""

__all__ = ["WidgetWindow"]

# External packages
import numpy as np

# Omniverse general API
import carb
import omni.ui as ui
from omni.ui import color as cl

from pegasus.simulator.ui.ui_delegate import UIDelegate
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, THUMBNAIL, WORLD_THUMBNAIL, WINDOW_TITLE


class WidgetWindow(ui.Window):

    # Design constants for the widgets
    LABEL_PADDING = 120
    BUTTON_HEIGHT = 50
    GENERAL_SPACING = 5

    WINDOW_WIDTH = 325
    WINDOW_HEIGHT = 850

    BUTTON_SELECTED_STYLE = {
        "Button": {
            "background_color": 0xFF5555AA,
            "border_color": 0xFF5555AA,
            "border_width": 2,
            "border_radius": 5,
            "padding": 5,
        }
    }

    BUTTON_BASE_STYLE = {
        "Button": {
            "background_color": cl("#292929"),
            "border_color": cl("#292929"),
            "border_width": 2,
            "border_radius": 5,
            "padding": 5,
        }
    }

    def __init__(self, delegate: UIDelegate, **kwargs):
        """
        Constructor for the Window UI widget of the extension. Receives as input a UIDelegate that implements
        all the callbacks to handle button clicks, drop-down menu actions, etc. (abstracting the interface between
        the logic of the code and the ui)
        """

        # Setup the base widget window
        super().__init__(
            WINDOW_TITLE, width=WidgetWindow.WINDOW_WIDTH, height=WidgetWindow.WINDOW_HEIGHT, visible=True, **kwargs
        )
        self.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

        # Setup the delegate that will bridge between the logic and the UI
        self._delegate = delegate

        # Bind the UI delegate to this window
        self._delegate.set_window_bind(self)

        # Auxiliar attributes for getting the transforms of the vehicle and the camera from the UI
        self._camera_transform_models = []
        self._vehicle_transform_models = []

        # Build the actual window UI
        self._build_window()

    def destroy(self):

        # Clear the world and the stage correctly
        self._delegate.on_clear_scene()

        # It will destroy all the children
        super().destroy()

    def _build_window(self):

        # Define the UI of the widget window
        with self.frame:
        
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON, vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON):

                # Vertical Stack of menus
                with ui.VStack():

                    # Create a frame for selecting which scene to load
                    self._scene_selection_frame()
                    ui.Spacer(height=5)

                    # Create a frame for selecting which vehicle to load in the simulation environment
                    self._robot_selection_frame()
                    ui.Spacer(height=5)

                    # Create a frame for selecting the camera position, and what it should point torwards to
                    self._viewport_camera_frame()
                    ui.Spacer()

    def _scene_selection_frame(self):
        """
        Method that implements a dropdown menu with the list of available simulation environemts for the vehicle
        """

        # Frame for selecting the simulation environment to load
        with ui.CollapsableFrame("Scene Selection"):
            with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                ui.Spacer(height=WidgetWindow.GENERAL_SPACING)

                # Iterate over all existing pre-made worlds bundled with this extension
                with ui.HStack():
                    ui.Label("World Assets", width=WidgetWindow.LABEL_PADDING, height=10.0)

                    # Combo box with the available environments to select from
                    dropdown_menu = ui.ComboBox(0, height=10, name="environments")
                    for environment in SIMULATION_ENVIRONMENTS:
                        dropdown_menu.model.append_child_item(None, ui.SimpleStringModel(environment))

                    # Allow the delegate to know which option was selected in the dropdown menu
                    self._delegate.set_scene_dropdown(dropdown_menu.model)

                ui.Spacer(height=0)

                # UI to configure the default latitude, longitude and altitude coordinates
                with ui.CollapsableFrame("Geographic Coordinates", collapsed=False):
                    with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                        with ui.HStack():

                            # Latitude
                            ui.Label("Latitude", name="label", width=WidgetWindow.LABEL_PADDING-50)
                            latitude_field = ui.FloatField(name="latitude", precision=6)
                            latitude_field.model.set_value(self._delegate._latitude)
                            self._delegate.set_latitude_field(latitude_field.model)
                            ui.Circle(name="transform", width=20, height=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED)

                            # Longitude
                            ui.Label("Longitude", name="label", width=WidgetWindow.LABEL_PADDING-50)
                            longitude_field = ui.FloatField(name="longitude", precision=6)
                            longitude_field.model.set_value(self._delegate._longitude)
                            self._delegate.set_longitude_field(longitude_field.model)
                            ui.Circle(name="transform", width=20, height=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED)

                            # Altitude
                            ui.Label("Altitude", name="label", width=WidgetWindow.LABEL_PADDING-50)
                            altitude_field = ui.FloatField(name="altitude", precision=6)
                            altitude_field.model.set_value(self._delegate._altitude)
                            self._delegate.set_altitude_field(altitude_field.model)
                            ui.Circle(name="transform", width=20, height=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED)

                        with ui.HStack():
                            ui.Button("Set", enabled=True, clicked_fn=self._delegate.on_set_new_global_coordinates)
                            ui.Button("Reset", enabled=True, clicked_fn=self._delegate.on_reset_global_coordinates)
                            ui.Button("Make Default", enabled=True, clicked_fn=self._delegate.on_set_new_default_global_coordinates)

                ui.Spacer(height=0)

                with ui.HStack():
                    # Add a thumbnail image to have a preview of the world that is about to be loaded
                    with ui.ZStack(width=WidgetWindow.LABEL_PADDING, height=WidgetWindow.BUTTON_HEIGHT * 2):
                        ui.Rectangle()
                        ui.Image(
                            WORLD_THUMBNAIL,
                            fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                            alignment=ui.Alignment.LEFT_CENTER,
                        )

                    ui.Spacer(width=WidgetWindow.GENERAL_SPACING)

                    with ui.VStack():
                        # Button for loading a desired scene
                        ui.Button(
                            "Load Scene",
                            height=WidgetWindow.BUTTON_HEIGHT,
                            clicked_fn=self._delegate.on_load_scene,
                            style=WidgetWindow.BUTTON_BASE_STYLE,
                        )

                        # Button to reset the stage
                        ui.Button(
                            "Clear Scene",
                            height=WidgetWindow.BUTTON_HEIGHT,
                            clicked_fn=self._delegate.on_clear_scene,
                            style=WidgetWindow.BUTTON_BASE_STYLE,
                        )

    def _robot_selection_frame(self):
        """
        Method that implements a frame that allows the user to choose which robot that is about to be spawned
        """

        # Auxiliary function to handle the "switch behaviour" of the buttons that are used to choose between a px4 or ROS2 backend
        def handle_px4_ros_switch(self, px4_button, ros2_button, button):

            # Handle the UI of both buttons switching of and on (To make it prettier)
            if button == "px4":
                px4_button.enabled = False
                ros2_button.enabled = True
                px4_button.set_style(WidgetWindow.BUTTON_SELECTED_STYLE)
                ros2_button.set_style(WidgetWindow.BUTTON_BASE_STYLE)
            else:
                px4_button.enabled = True
                ros2_button.enabled = False
                ros2_button.set_style(WidgetWindow.BUTTON_SELECTED_STYLE)
                px4_button.set_style(WidgetWindow.BUTTON_BASE_STYLE)

            # Handle the logic of switching between the two operating modes
            self._delegate.set_streaming_backend(button)

        # --------------------------
        # Function UI starts here
        # --------------------------

        # Frame for selecting the vehicle to load
        with ui.CollapsableFrame(title="Vehicle Selection"):
            with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                ui.Spacer(height=WidgetWindow.GENERAL_SPACING)
                # Iterate over all existing robots in the extension
                with ui.HStack():
                    ui.Label("Vehicle Model", name="label", width=WidgetWindow.LABEL_PADDING)

                    # Combo box with the available vehicles to select from
                    dropdown_menu = ui.ComboBox(0, height=10, name="robots")
                    for robot in ROBOTS:
                        dropdown_menu.model.append_child_item(None, ui.SimpleStringModel(robot))
                    self._delegate.set_vehicle_dropdown(dropdown_menu.model)

                with ui.HStack():
                    ui.Label("Vehicle ID", name="label", width=WidgetWindow.LABEL_PADDING)
                    vehicle_id_field = ui.IntField()
                    self._delegate.set_vehicle_id_field(vehicle_id_field.model)

                # Add a frame transform to select the position of where to place the selected robot in the world
                self._transform_frame()

                ui.Label("Streaming Backend")
                
                with ui.HStack():
                    # Add a thumbnail image to have a preview of the world that is about to be loaded
                    with ui.ZStack(width=WidgetWindow.LABEL_PADDING, height=WidgetWindow.BUTTON_HEIGHT * 2):
                        ui.Rectangle()
                        ui.Image(
                            THUMBNAIL, fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT, alignment=ui.Alignment.LEFT_CENTER
                        )

                    ui.Spacer(width=WidgetWindow.GENERAL_SPACING)
                    with ui.VStack():
                        # Buttons that behave like switches to choose which network interface to use to simulate the control of the vehicle
                        px4_button = ui.Button(
                            "PX4",
                            height=WidgetWindow.BUTTON_HEIGHT * 2,
                            style=WidgetWindow.BUTTON_SELECTED_STYLE,
                            enabled=False,
                        )
                        ros2_button = ui.Button(
                            "ROS 2",
                            height=WidgetWindow.BUTTON_HEIGHT,
                            style=WidgetWindow.BUTTON_BASE_STYLE,
                            enabled=True,
                            visible=False
                        )

                        # Set the auxiliary function to handle the switch between both backends
                        px4_button.set_clicked_fn(lambda: handle_px4_ros_switch(self, px4_button, ros2_button, "px4"))
                        ros2_button.set_clicked_fn(lambda: handle_px4_ros_switch(self, px4_button, ros2_button, "ros"))

                # UI to configure the PX4 settings
                with ui.CollapsableFrame("PX4 Configurations", collapsed=False):
                    with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                        with ui.HStack():
                            ui.Label("Auto-launch PX4", name="label", width=WidgetWindow.LABEL_PADDING - 20)
                            px4_checkbox = ui.CheckBox()
                            px4_checkbox.model.set_value(self._delegate._autostart_px4)
                            self._delegate.set_px4_autostart_checkbox(px4_checkbox.model)

                        with ui.HStack():
                            ui.Label("PX4 Path", name="label", width=WidgetWindow.LABEL_PADDING - 20)
                            px4_path_field = ui.StringField(name="px4_path", width=300)
                            px4_path_field.model.set_value(self._delegate._px4_dir)
                            self._delegate.set_px4_directory_field(px4_path_field.model)

                            ui.Button("Reset", enabled=True, clicked_fn=self._delegate.on_reset_px4_path)
                            ui.Button("Make Default", enabled=True, clicked_fn=self._delegate.on_set_new_default_px4_path)

                        with ui.HStack():
                            ui.Label("PX4 airframe", name="label", width=WidgetWindow.LABEL_PADDING - 20)
                            px4_airframe_field = ui.StringField(name="px4_model")
                            px4_airframe_field.model.set_value(self._delegate._px4_airframe)
                            self._delegate.set_px4_airframe_field(px4_airframe_field.model)

                # Button to load the drone
                ui.Button(
                    "Load Vehicle",
                    height=WidgetWindow.BUTTON_HEIGHT,
                    clicked_fn=self._delegate.on_load_vehicle,
                    style=WidgetWindow.BUTTON_BASE_STYLE,
                )

    def _viewport_camera_frame(self):
        """
        Method that implements a frame that allows the user to choose what is the viewport camera pose easily
        """

        all_axis = ["X", "Y", "Z"]
        colors = {"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}
        default_values = [5.0, 5.0, 5.0]
        target_default_values = [0.0, 0.0, 0.0]

        # Frame for setting the camera to visualize the vehicle in the simulator viewport
        with ui.CollapsableFrame("Viewport Camera"):
            with ui.VStack(spacing=8):
                ui.Spacer(height=0)

                # Iterate over the position and rotation menus
                with ui.HStack():
                    with ui.HStack():
                        ui.Label("Position", name="transform", width=50, height=20)
                        ui.Spacer()
                    # Fields X, Y and Z
                    for axis, default_value in zip(all_axis, default_values):
                        with ui.HStack():
                            with ui.ZStack(width=15):
                                ui.Rectangle(
                                    width=15,
                                    height=20,
                                    style={
                                        "background_color": colors[axis],
                                        "border_radius": 3,
                                        "corner_flag": ui.CornerFlag.LEFT,
                                    },
                                )
                                ui.Label(axis, height=20, name="transform_label", alignment=ui.Alignment.CENTER)
                            float_drag = ui.FloatDrag(name="transform", min=-1000000, max=1000000, step=0.01)
                            float_drag.model.set_value(default_value)
                            # Save the model of each FloatDrag such that we can access its values later on
                            self._camera_transform_models.append(float_drag.model)
                            ui.Circle(
                                name="transform", width=20, height=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED
                            )

                # Iterate over the position and rotation menus
                with ui.HStack():
                    with ui.HStack():
                        ui.Label("Target", name="transform", width=50, height=20)
                        ui.Spacer()
                    # Fields X, Y and Z
                    for axis, default_value in zip(all_axis, target_default_values):
                        with ui.HStack():
                            with ui.ZStack(width=15):
                                ui.Rectangle(
                                    width=15,
                                    height=20,
                                    style={
                                        "background_color": colors[axis],
                                        "border_radius": 3,
                                        "corner_flag": ui.CornerFlag.LEFT,
                                    },
                                )
                                ui.Label(axis, height=20, name="transform_label", alignment=ui.Alignment.CENTER)
                            float_drag = ui.FloatDrag(name="transform", min=-1000000, max=1000000, step=0.01)
                            float_drag.model.set_value(default_value)
                            # Save the model of each FloatDrag such that we can access its values later on
                            self._camera_transform_models.append(float_drag.model)
                            ui.Circle(
                                name="transform", width=20, height=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED
                            )

                # Button to set the camera view
                ui.Button(
                    "Set Camera Pose",
                    height=WidgetWindow.BUTTON_HEIGHT,
                    clicked_fn=self._delegate.on_set_viewport_camera,
                    style=WidgetWindow.BUTTON_BASE_STYLE,
                )
                ui.Spacer()

    def _transform_frame(self):
        """
        Method that implements a transform frame to translate and rotate an object that is about to be spawned
        """

        components = ["Position", "Rotation"]
        all_axis = ["X", "Y", "Z"]
        colors = {"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}
        default_values = [0.0, 0.0, 0.1]

        with ui.CollapsableFrame("Position and Orientation"):
            with ui.VStack(spacing=8):

                ui.Spacer(height=0)

                # Iterate over the position and rotation menus
                for component in components:
                    with ui.HStack():
                        with ui.HStack():
                            ui.Label(component, name="transform", width=50)
                            ui.Spacer()
                        # Fields X, Y and Z
                        for axis, default_value in zip(all_axis, default_values):
                            with ui.HStack():
                                with ui.ZStack(width=15):
                                    ui.Rectangle(
                                        width=15,
                                        height=20,
                                        style={
                                            "background_color": colors[axis],
                                            "border_radius": 3,
                                            "corner_flag": ui.CornerFlag.LEFT,
                                        },
                                    )
                                    ui.Label(axis, name="transform_label", alignment=ui.Alignment.CENTER)
                                if component == "Position":
                                    float_drag = ui.FloatDrag(name="transform", min=-1000000, max=1000000, step=0.01)
                                    float_drag.model.set_value(default_value)
                                else:
                                    float_drag = ui.FloatDrag(name="transform", min=-180.0, max=180.0, step=0.01)
                                # Save the model of each FloatDrag such that we can access its values later on
                                self._vehicle_transform_models.append(float_drag.model)
                                ui.Circle(name="transform", width=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED)
                ui.Spacer(height=0)

    # ------------------------------------------------------------------------------------------------
    # TODO - optimize the reading of values from the transform widget. This could be one function only
    # ------------------------------------------------------------------------------------------------

    def get_selected_vehicle_attitude(self):

        # Extract the vehicle desired position and orientation for spawning
        if len(self._vehicle_transform_models) == 6:
            vehicle_pos = np.array([self._vehicle_transform_models[i].get_value_as_float() for i in range(3)])
            vehicel_orientation = np.array(
                [self._vehicle_transform_models[i].get_value_as_float() for i in range(3, 6)]
            )
            return vehicle_pos, vehicel_orientation

        return None, None

    def get_selected_camera_pos(self):
        """
        Method that returns the currently selected camera position in the camera transform widget
        """

        # Extract the camera desired position and the target it is pointing to
        if len(self._camera_transform_models) == 6:
            camera_pos = np.array([self._camera_transform_models[i].get_value_as_float() for i in range(3)])
            camera_target = np.array([self._camera_transform_models[i].get_value_as_float() for i in range(3, 6)])
            return camera_pos, camera_target

        return None, None

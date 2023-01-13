#!/usr/bin/env python
import carb
import omni.ui as ui

from pegasus_isaac.ui.ui_delegate import UIDelegate
from pegasus_isaac.params import ROBOTS

class WidgetWindow:

    # Design constants for the widgets
    LABEL_PADDING = 120

    def __init__(self, delegate: UIDelegate):
        """
        Constructor for the Window UI widget of the extension. Receives as input a UIDelegate that implements
        all the callbacks to handle button clicks, drop-down menu actions, etc. (abstracting the interface between
        the logic of the code and the ui)
        """

        self._window = None
        self._delegate = delegate

    def shutdown(self):
        """Should be called when the extesion is unloaded"""
        # --------------------
        # According to NVidia:
        # --------------------
        # Unfortunatley, the member variables are not destroyed when the extension is unloaded. We need to do it
        # automatically. Usually, it's OK because the Python garbage collector will eventually destroy everythigng. But
        # we need the images to be destroyed right now because Kit know nothing about Python garbage collector and it
        # will fire warning that texture is not destroyed.
        self._window = None

    def build_window(self):

        self._window = ui.Window("Pegasus Simulation", width=450, height=800, visible=True)
        #self._window.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        #self._window.setPosition(0.0, 0.0)

        carb.log_warn("drawing")
        
        # Define the UI of the widget window
        with self._window.frame:
            
            # Vertical Stack of menus
            with ui.VStack():
                
                # Frame for selecting the simulation environment to load
                with ui.CollapsableFrame("Scene Selection"):
                
                    # Button for loading a desired scene
                    ui.Button("Load Scene", clicked_fn=self._delegate.on_load_scene)

                    # Button to reset the stage
                    ui.Button("Clear Scene", clicked_fn=self._delegate.on_clear_scene)

                # Frame for selecting the vehicle to spawn in the simulation environment
                with ui.CollapsableFrame("Vehicle Selection"):

                    # Create a frame for selecting which vehicle to load in the simulation environment
                    self._robot_selection_frame()
                    
                    # Create a transform frame to choose where to spawn the vehicle
                    self._transform_frame()

                    # Button to load the drone
                    ui.Button("Load Vehicle", clicked_fn=self._delegate.on_load_vehicle)
                
                # Frame for setting the camera to visualize the vehicle in the simulator viewport
                with ui.CollapsableFrame("Viewport Camera"):

                    # Create a transform frame to choose where to spawn the vehicle
                    self._transform_frame()

                    # Button to set the camera view
                    ui.Button("Set Camera Pose", clicked_fn=self._delegate.on_set_viewport_camera) 

    def _robot_selection_frame(self):
        """
        Method that implements a frame that allows the user to choose which robot that is about to be spawned
        """

        with ui.Frame():
            with ui.ZStack():
                ui.Rectangle(name="frame_background")
                with ui.VStack(height=0, spacing=5, name="frame_v_stack"):
                    ui.Spacer(height=5)
                    # Iterate over all existing robots in the extension
                    with ui.HStack():
                        ui.Label("Vehicle Model", name="label", width=WidgetWindow.LABEL_PADDING)
                        robot_combo_box = ui.ComboBox(0, height=10, name="robots")
                        for robot in ROBOTS:
                            robot_combo_box.model.append_child_item(None, ui.SimpleStringModel(robot))                  

    def _transform_frame(self):
        """
        Method that implements a transform frame to translate and rotate an object that is about to be spawned
        """

        components=["Position", "Rotation"]
        all_axis=["X", "Y", "Z"]
        colors={"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}

        with ui.CollapsableFrame(title="Transform"):
            with ui.VStack(spacing=8):
                ui.Spacer(height=0)

                # Iterate over the position and rotation menus
                for component in components:
                    with ui.HStack():
                        with ui.HStack(width=WidgetWindow.LABEL_PADDING):
                            ui.Label(component, name="transform", width=50)
                            ui.Spacer()
                        # Fields X, Y and Z
                        for axis in all_axis:
                            with ui.HStack():
                                with ui.ZStack(width=15):
                                    ui.Rectangle(width=15, height=20, style={"background_color": colors[axis], "border_radius": 3, "corner_flag": ui.CornerFlag.LEFT})
                                    ui.Label(axis, name="transform_label", alignment=ui.Alignment.CENTER)
                                if component == "Position":
                                    ui.FloatDrag(name="transform", min=-1000000, max=1000000, step=0.01)
                                else:
                                    ui.FloatDrag(name="transform", min=-180.0, max=180.0, step=0.01)
                                ui.Circle(name="transform", width=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED)
                ui.Spacer(height=0)    

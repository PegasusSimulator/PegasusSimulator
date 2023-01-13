#!/usr/bin/env python
import carb

import omni.ui as ui
from omni.isaac.core import World

class UIDelegate:
    """
    Object that will interface between the logic/dynamic simulation part of the extension and the Widget UI
    """

    def __init__(self, world: World, world_settings):
        
        self._world = world
        self._world_settings = world_settings

        # Attribute that holds the currently selected scene from the dropdown menu
        self._scene_dropdown: ui.AbstractItemModel = None
        
        # Attribute that hold the currently selected vehicle from the dropdown menu
        self._vehicle_dropdown: ui.AbstractItemModel = None

    def set_scene_dropdown(self, scene_dropdown_model: ui.AbstractItemModel):
        self._scene_dropdown = scene_dropdown_model

    def set_vehicle_dropdown(self, vehicle_dropdown_model: ui.AbstractItemModel):
        self._vehicle_dropdown = vehicle_dropdown_model
    
    """
    ---------------------------------------------------------------------
    Callbacks to handle user interaction with the extension widget window
    ---------------------------------------------------------------------
    """

    def on_load_scene(self):
        """
        Method that should be invoked when the button to load the selected world is pressed
        """
        carb.log_warn("New scene has been loaded")

        # Check if a scene is selected in the drop-down menu
        if self._scene_dropdown is not None:

            # Get the id of the selected environment from the list
            environemnt_index = self._scene_dropdown.get_item_value_model().as_int
            
            carb.log_warn(self._scene_dropdown.get_item_value_model().as_string)

            # Load the scene
            # TODO

    def on_clear_scene(self):
        """
        Method that should be invoked when the clear world button is pressed
        """
        carb.log_warn("Current scene and its vehicles has been deleted")

    def on_load_vehicle(self):
        """
        Method that should be invoked when the button to load the selected vehicle is pressed
        """
        carb.log_warn("A new vehicle has been loaded")

        # Check fi a vehicle is selected in the drop-down menu
        if self._vehicle_dropdown is not None:

            # Get the id of the selected vehicle from the list
            vehicle_index = self._vehicle_dropdown.get_item_value_model().as_int

            carb.log_warn(self._vehicle_dropdown.get_item_value_model().as_string)
    
    def on_set_viewport_camera(self):
        """
        Method that should be invoked when the button to set the viewport camera pose is pressed
        """
        carb.log_warn("The viewport camera pose has been adjusted")

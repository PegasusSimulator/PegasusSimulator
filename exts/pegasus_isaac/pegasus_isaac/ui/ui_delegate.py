#!/usr/bin/env python
import carb
from omni.isaac.core import World

class UIDelegate:
    """
    Object that will interface between the logic/dynamic simulation part of the extension and the Widget UI
    """

    def __init__(self, world: World, world_settings):
        
        self._world = world
        self._world_settings = world_settings

    def on_load_scene(self):
        """
        Method that should be invoked when the button to load the selected world is pressed
        """
        carb.log_warn("New scene has been loaded")

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
    
    def on_set_viewport_camera(self):
        """
        Method that should be invoked when the button to set the viewport camera pose is pressed
        """
        carb.log_warn("The viewport camera pose has been adjusted")

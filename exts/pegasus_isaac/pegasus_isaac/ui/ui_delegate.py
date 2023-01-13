#!/usr/bin/env python
import carb
from omni.isaac.core import World

class UIDeletegate:
    """
    Object that will interface between the logic/dynamic simulation part of the extension and the Widget UI
    """

    def __init__(self, world: World, world_settings):
        
        self._world = world
        self._world_settings = world_settings

    def on_shutdown(self):
        """
        Callback called when the extension is shutdown
        """
        carb.log_info("Pegasus Isaac extension shutdown")
        
        self.clear_world()

    def on_load_scene(self):
        """
        Method that should be invoked when the button to load the selected world is pressed
        """
        pass

    def on_clear_scene(self):
        """
        Method that should be invoked when the clear world button is pressed
        """

    def on_load_vehicle(self):
        """
        Method that should be invoked when the button to load the selected vehicle is pressed
        """
        pass
    
    def on_set_viewport_camera(self):
        """
        Method that should be invoked when the button to set the viewport camera pose is pressed
        """
        pass

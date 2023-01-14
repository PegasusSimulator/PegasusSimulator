# Python garbage collenction and asyncronous API
import gc
import asyncio

# External packages
import numpy as np

# Omniverse general API
import carb
import omni.ext
import omni.kit.ui
import omni.kit.app
import omni.ui as ui

# Isaac Speficic extensions API
from omni.isaac.core import World
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.stage import create_new_stage_async, set_stage_up_axis, clear_stage, add_reference_to_stage, get_current_stage

# Pegasus Extension Files
from pegasus_isaac.params import ROBOTS, DEFAULT_WORLD_SETTINGS, MENU_PATH

# Quadrotor vehicle
from pegasus_isaac.logic.vehicles.quadrotor import Quadrotor

# Setting up the UI for the extension's Widget
from pegasus_isaac.ui.ui_window import WidgetWindow
from pegasus_isaac.ui.ui_delegate import UIDelegate

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class Pegasus_isaacExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        
        carb.log_info("Pegasus extension startup")

        # Save the extension id
        self._ext_id = ext_id

        # Get the handle for the extension manager
        self._extension_manager = omni.kit.app.get_app().get_extension_manager()      
        
        # Basic world configurations
        self._world_settings = DEFAULT_WORLD_SETTINGS
        self._world: World = World(**self._world_settings)

        # Create the UI of the app and its manager
        self.ui_delegate = UIDelegate(self._world, self._world_settings)
        self.ui_window = None

        # Add the extension to the editor menu inside isaac sim
        self.editor_menu = omni.kit.ui.get_editor_menu()
        if self.editor_menu:
            self._menu = self.editor_menu.add_item(MENU_PATH, self.show_window, toggle=True, value=True)

    def show_window(self, menu, value):
        """
        Method that controls whether a widget window is created or not
        """
        if value is not None and value == True:
            # Create a window 
            self.ui_window = WidgetWindow(self.ui_delegate)
            self.ui_window.set_visibility_changed_fn(self._visibility_changed_fn)
        
        carb.log_warn("showing window")
        carb.log_warn(menu)
        carb.log_warn(value)

    def _visibility_changed_fn(self, visible):
        """        
        This method is invoked when the user pressed the "X" to close the extension window
        """
        if not visible:
            # Destroy the window, because we create a new one in the show window method
            asyncio.ensure_future(self._destroy_window_async())
        
    async def _destroy_window_async(self):
        
        # Wait one frame before it gets destructed (from NVidia example)
        await omni.kit.app.get_app().next_update_async()

        # Destroy the window UI if it exists
        if self.ui_window:
            self.ui_window.destroy()
            self.ui_window = None

                    
    def set_world_settings(self, physics_dt=None, stage_units_in_meters=None, rendering_dt=None):
        """
        Set the current world settings to the pre-defined settings
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

    def on_shutdown(self):
        """
        Callback called when the extension is shutdown
        """
        carb.log_info("Pegasus Isaac extension shutdown")
        
        self.clear_world()


    def change_visibility(self, visible):
        """
        Method that is called when the visibility of the extension is changed
        """
        if not visible:
            self.on_shutdown()

    def check_ros_extension(self):
        """
        Method that checks which ROS extension is installed.
        """
        
        version = ""
        
        if self._ext_manager.is_extension_enabled("omni.isaac.ros_bridge"):
            version = "ros"
        elif self._ext_manager.is_extension_enabled("omni.isaac.ros2_bridge"):
            version = "ros2"
        else:
            carb.log_warn("Neither extension 'omni.isaac.ros_bridge' nor 'omni.isaac.ros2_bridge' is enabled")
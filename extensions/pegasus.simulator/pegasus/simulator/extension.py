"""
| File: extension.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Implements the Pegasus_SimulatorExtension which omni.ext.IExt that is created when this class is enabled. In turn, this class initializes the extension widget.
"""
__all__ = ["Pegasus_SimulatorExtension"]

# Python garbage collenction and asyncronous API
import gc
import asyncio
from functools import partial
from threading import Timer

# Omniverse general API
import pxr
import carb
import omni.ext
import omni.usd
import omni.kit.ui
import omni.kit.app
import omni.ui as ui

from omni.kit.viewport.utility import get_active_viewport

# Pegasus Extension Files and API
from pegasus.simulator.params import MENU_PATH, WINDOW_TITLE
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Setting up the UI for the extension's Widget
from pegasus.simulator.ui.ui_window import WidgetWindow
from pegasus.simulator.ui.ui_delegate import UIDelegate

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class Pegasus_SimulatorExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):

        carb.log_info("Pegasus Simulator is starting up")

        # Save the extension id
        self._ext_id = ext_id

        # Create the UI of the app and its manager
        self.ui_delegate = None
        self.ui_window = None

        # Start the extension backend
        self._pegasus_sim = PegasusInterface()

        # Check if we already have a stage loaded (when using autoload feature, it might not be ready yet)
        # This is a limitation of the simulator, and we are doing this to make sure that the 
        # extension does no crash when using the GUI with autoload feature
        # If autoload was not enabled, and we are enabling the extension from the Extension widget, then 
        # we will always have a state open, and the auxiliary timer will never run
        if omni.usd.get_context().get_stage_state() != omni.usd.StageState.CLOSED:
            self._pegasus_sim.initialize_world()
        else:
            # We need to create a timer to check until the window is properly open and the stage created. This is a limitation
            # of the current Isaac Sim simulator and the way it loads extensions :(
            self.autoload_helper()

        # Add the ability to show the window if the system requires it (QuickLayout feature)
        ui.Workspace.set_show_window_fn(WINDOW_TITLE, partial(self.show_window, None))

        # Add the extension to the editor menu inside isaac sim
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            self._menu = editor_menu.add_item(MENU_PATH, self.show_window, toggle=True, value=True)

        # Show the window (It call the self.show_window)
        ui.Workspace.show_window(WINDOW_TITLE, show=True)

    def autoload_helper(self):
        
        # Check if we already have a viewport and a camera of interest
        if get_active_viewport() != None and type(get_active_viewport().stage) == pxr.Usd.Stage and str(get_active_viewport().stage.GetPrimAtPath("/OmniverseKit_Persp")) != "invalid null prim":
            self._pegasus_sim.initialize_world()
        else:
            Timer(1.0, self.autoload_helper).start()

    def show_window(self, menu, show):
        """
        Method that controls whether a widget window is created or not
        """

        if show == True:

            # Create a window and its delegate
            self.ui_delegate = UIDelegate()
            self.ui_window = WidgetWindow(self.ui_delegate)
            self.ui_window.set_visibility_changed_fn(self._visibility_changed_fn)

        # If we have a window and we are not supposed to show it, then change its visibility
        elif self.ui_window:
            self.ui_window.visible = False

    def _visibility_changed_fn(self, visible):
        """
        This method is invoked when the user pressed the "X" to close the extension window
        """

        # Update the Isaac sim menu visibility
        self._set_menu(visible)

        if not visible:
            # Destroy the window, because we create a new one in the show window method
            asyncio.ensure_future(self._destroy_window_async())

    def _set_menu(self, visible):
        """
        Method that updates the isaac sim ui menu to create the Widget window on and off
        """
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            editor_menu.set_value(MENU_PATH, visible)

    async def _destroy_window_async(self):

        # Wait one frame before it gets destructed (from NVidia example)
        await omni.kit.app.get_app().next_update_async()

        # Destroy the window UI if it exists
        if self.ui_window:
            self.ui_window.destroy()
            self.ui_window = None

    def on_shutdown(self):
        """
        Callback called when the extension is shutdown
        """
        carb.log_info("Pegasus Isaac extension shutdown")

        # Destroy the isaac sim menu object
        self._menu = None

        # Destroy the window
        if self.ui_window:
            self.ui_window.destroy()
            self.ui_window = None

        # Destroy the UI delegate
        if self.ui_delegate:
            self.ui_delegate = None

        # De-register the function taht shows the window from the isaac sim ui
        ui.Workspace.set_show_window_fn(WINDOW_TITLE, None)

        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            editor_menu.remove_item(MENU_PATH)

        # Call the garbage collector
        gc.collect()

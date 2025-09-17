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
import omni.timeline

from omni.kit.viewport.utility import get_active_viewport
from isaacsim.core.api import World

# Pegasus Extension Files and API
from pegasus.simulator.params import MENU_PATH, WINDOW_TITLE, DEFAULT_WORLD_SETTINGS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from pxr import Usd, UsdGeom

from carb.eventdispatcher import get_eventdispatcher

# Setting up the UI for the extension's Widget
from pegasus.simulator.ui.ui_window import WidgetWindow
from pegasus.simulator.ui.ui_delegate import UIDelegate

from isaacsim.core.utils.stage import (
    clear_stage,
    create_new_stage,
    create_new_stage_async,
    get_current_stage,
    set_stage_units,
    set_stage_up_axis,
    update_stage_async,
)


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class Pegasus_SimulatorExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):

        carb.log_info("Pegasus Simulator is starting up")

        # Create the Pegasus interface that manages the simulation
        self.pg = PegasusInterface()

        # Import and register the OmniGraph nodes

        # Save the extension id
        self._ext_id = ext_id

        # Create the UI of the app and its manager
        self.ui_delegate = None
        self.ui_window = None

        # Add the ability to show the window if the system requires it (QuickLayout feature)
        ui.Workspace.set_show_window_fn(WINDOW_TITLE, partial(self.show_window, None))

        # Add the extension to the editor menu inside isaac sim
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            self._menu = editor_menu.add_item(
                MENU_PATH, self.show_window, toggle=True, value=True
            )

        # Show the window (It call the self.show_window)
        ui.Workspace.show_window(WINDOW_TITLE, show=True)

        # Subscribe to stage events
        print("Subscribing to stage events")

        stage_event_stream = omni.usd.get_context().get_stage_event_stream()

        # Create a subscription to the event stream
        # The subscription will last as long as the 'subscription' variable is in scope
        self.stage_subscription = stage_event_stream.create_subscription_to_pop(
            self.on_stage_event, name="my_stage_listener"
        )
    
    def on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.OPENED):
            stage = omni.usd.get_context().get_stage()
            if stage:
                print("USD Stage opened:", stage.GetRootLayer().realPath)
                # Add your custom logic here
                # For example, to print all prims in the stage:
                # for prim in stage.Traverse():
                #     print(prim.GetPath())
                # set the pegasus world to use this stage
                asyncio.ensure_future(self.set_pegasus_world_and_physics())
    
    async def set_pegasus_world_and_physics(self):
        # set the pegasus world to use the current stage
        self.pg._world = World.instance()

        # if the world was None even after we set it, then initialize it
        if self.pg.world is None:
            carb.log_warn("The world is None")
            self.pg._world_settings = DEFAULT_WORLD_SETTINGS
            # we need to wait for this to be initialized before continuing
            await self.pg.initialize_world()
            carb.log_info("PEGASUS WORLD INITIALIZED")
            print("PEGASUS WORLD INITIALIZED", self.pg._world)
        # else if there was a world but it was missing the physics, then add physics
        elif self.pg.world.get_physics_context() is None:
            await self.pg.world.initialize_simulation_context_async()

        # check if the world settings match the required ones
        if (
            self.pg.world.get_physics_context() is not None
            and self.pg.world.get_physics_dt()
            != self.pg._world_settings["physics_dt"]
            or self.pg.world.get_rendering_dt()
            != self.pg._world_settings["rendering_dt"]
        ):
            carb.log_warn(
                "The physics and rendering dt of the world don't match Pegasus required settings. "
            )
            print(
                f"World physics dt: {self.pg.world.get_physics_dt()}, World rendering dt: {self.pg.world.get_rendering_dt()}"
            )
            print("Updating the physics and rendering dt of the world")
            self.pg.world.set_simulation_dt(physics_dt=self.pg._world_settings["physics_dt"], rendering_dt=self.pg._world_settings["rendering_dt"])

        if (
            UsdGeom.GetStageMetersPerUnit(self.pg.world.stage)
            != self.pg._world_settings["stage_units_in_meters"]
        ):
            carb.log_warn(
                "The stage units don't match Pegasus required settings. "
            )
            print(
                f"Stage units: {self.pg.world.stage.GetRootLayer().GetMetersPerUnit()}"
            )
            print("Updating the stage units")
            set_stage_units(self.pg.world.stage, self.pg._world_settings["stage_units_in_meters"])


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

#!/usr/bin/env python
"""
| File: 0_template_app.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as a template on how to build a clean and simple Isaac Sim based standalone App.
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf
from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent
from isaacsim.core.rendering_manager import RenderingManager, RenderingEvent
import isaacsim.core.experimental.utils.app as app_utils

class Template:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the template App and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Set up simulation with a default physics dt of 1/60 s
        SimulationManager.set_physics_dt(1.0 / 60.0)

        # Add a simple ground plane via USD physics (large flat cube with collision)
        stage = omni.usd.get_context().get_stage()
        ground = UsdGeom.Cube.Define(stage, "/World/groundPlane")
        ground.GetSizeAttr().Set(150.0)
        xform = UsdGeom.Xformable(ground.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.5))
        xform.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 0.01))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        # Register example physics callback
        self._cb_physics = SimulationManager.register_callback(
            self.physics_callback, SimulationEvent.PHYSICS_POST_STEP
        )

        # Register example render callback
        self._cb_render = RenderingManager.register_callback(
            RenderingEvent.NEW_FRAME, callback=self.render_callback
        )

        # Register simulation stopped callback (replaces the old timeline callback)
        self._cb_stop = SimulationManager.register_callback(
            self._on_sim_stopped, SimulationEvent.SIMULATION_STOPPED
        )

        # Auxiliar variable for the stop callback example
        self.stop_sim = False

    def physics_callback(self, dt: float, context=None):
        """An example physics callback. It will get invoked every physics step.

        Args:
            dt (float): The time difference between the previous and current function call, in seconds.
            context: Physics step context (unused).
        """
        carb.log_info("This is a physics callback. It is called every " + str(dt) + " seconds!")

    def render_callback(self, event):
        """An example render callback. It will get invoked for every rendered frame.

        Args:
            event: Rendering event from RenderingManager.
        """
        carb.log_info("This is a render callback. It is called every frame!")

    def _on_sim_stopped(self, *args):
        """Invoked when the simulation stops."""
        self.stop_sim = True

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            simulation_app.update()

        # Cleanup and stop
        carb.log_warn("Template Simulation App is closing.")
        SimulationManager.deregister_callback(self._cb_physics)
        SimulationManager.deregister_callback(self._cb_stop)
        RenderingManager.deregister_callback(self._cb_render)
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    template_app = Template()

    # Run the application loop
    template_app.run()

if __name__ == "__main__":
    main()

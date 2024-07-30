#!/usr/bin/env python
"""
| File: 9_people.py
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API to run a simulation
| where people move around in the world.
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
import omni.usd
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.utils.extensions import disable_extension, enable_extension

# Enable/disable ROS bridge extensions to keep only ROS2 Bridge
disable_extension("omni.isaac.ros_bridge")
enable_extension("omni.isaac.ros2_bridge")

EXTENSIONS_PEOPLE = [
    'omni.anim.people', 
    'omni.anim.navigation.bundle', 
    'omni.anim.timeline',
    'omni.anim.graph.bundle', 
    'omni.anim.graph.core', 
    'omni.anim.graph.ui',
    'omni.anim.retarget.bundle', 
    'omni.anim.retarget.core',
    'omni.anim.retarget.ui', 
    'omni.kit.scripting',
    'omni.graph.io',
    'omni.anim.curve.core',
    'omni.anim.skelJoint',
    'omni.anim.graph.schema',
    'omni.anim.retarget.core',
    'omni.replicator.agent.core'
]

for ext_people in EXTENSIONS_PEOPLE:
    enable_extension(ext_people)

import numpy as np

# Import the Pegasus API for simulating drones
# from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
# from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
# from pegasus.simulator.logic.people.person import Person
# from pegasus.simulator.logic.people.person_controller import PersonController
# from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig
# from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
# from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

import omni.anim.graph.core as ag

class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        #self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics,
        # spawning asset primitives, etc.
        #self.pg._world = World(**self.pg._world_settings)
        #self.world = self.pg.world
        omni.usd.get_context().open_stage("/home/marcelo/pegasus/Debug.usd", None)
        self.world = World()

        # Launch one of the worlds provided by NVIDIA
        #self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        #self.pg.load_asset(SIMULATION_ENVIRONMENTS["Curved Gridroom"], "/World/layout")

        import carb
        from omni.isaac.core.utils import prims
        from omni.anim.people import PeopleSettings
        from pxr import Gf, Sdf

        from omni.isaac.nucleus import get_assets_root_path

        # prim = prims.create_prim("/World/Female", "Xform", usd_path="/home/marcelo/pegasus/female.usd")
        # print(dir(ag))
        
        # Reset the simulation environment so that all articulations (aka robots) are initialized
        #self.world.reset()


        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            #character_graph = ag.get_character("/World/Characters/female_adult_business_02/female_adult_business_02/ManRoot/female_adult_business_02")
            #print(character_graph)

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()

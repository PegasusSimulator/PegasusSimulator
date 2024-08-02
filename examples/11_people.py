#!/usr/bin/env python
"""
| File: 10_people.py
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

from omni.isaac.core.utils.extensions import disable_extension, enable_extension

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
]

for ext_people in EXTENSIONS_PEOPLE:
    enable_extension(ext_people)

# -----------------------------------
# The actual script should start here
# -----------------------------------
# Enable/disable ROS bridge extensions to keep only ROS2 Bridge
disable_extension("omni.isaac.ros_bridge")
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

# Auxiliary scipy and numpy modules
import omni.timeline
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation

import omni.anim.graph.core as ag
import carb
from omni.isaac.core.utils import prims
from omni.anim.people import PeopleSettings
from pxr import Gf, Sdf
from omni.isaac.nucleus import get_assets_root_path

# These lines are needed to restart the USD stage and make sure that the people extension is loaded
import omni.usd
omni.usd.get_context().new_stage()

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

        # Instantiate the world object
        self.world = World()

        # Get the characeters spawned in the world
        setting_dict = carb.settings.get_settings()
        character_root_prim_path = setting_dict.get(PeopleSettings.CHARACTER_PRIM_PATH)
        people_asset_folder = get_assets_root_path() + "/Isaac/People/Characters"

        carb.log_warn(f"Character root prim path: {character_root_prim_path}")
        carb.log_warn(f"People Assets folder: {people_asset_folder}")

        prims.create_prim(character_root_prim_path, "Xform")

        prim = prims.create_prim("/World/Characters" + "/Biped_Setup", "Xform", usd_path=people_asset_folder + "/Biped_Setup.usd")
        prim.GetAttribute("visibility").Set("invisible")

        prim2 = prims.create_prim("/World/Characters" + "/female_adult_business_02", "Xform", usd_path=people_asset_folder + "/F_Business_02/F_Business_02.usd")

        if type(prim2.GetAttribute("xformOp:orient").Get()) == Gf.Quatf:
            prim2.GetAttribute("xformOp:orient").Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1), 0.0).GetQuat()))
        else:
            prim2.GetAttribute("xformOp:orient").Set(Gf.Rotation(Gf.Vec3d(0,0,1), 0.0).GetQuat())

        # Apply the animation graphto the character
        omni.kit.commands.execute("ApplyAnimationGraphAPICommand", paths=[Sdf.Path("/World/Characters/female_adult_business_02/female_adult_business_02/ManRoot/female_adult_business_02")], animation_graph_path=Sdf.Path("/World/Characters/Biped_Setup/CharacterAnimation/AnimationGraph"))
        omni.kit.commands.execute("AnimGraphUIRefreshPropertyWindowCommand")

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

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

            # Get the animation graphso that we can apply actions
            character_graph = ag.get_character("/World/Characters/female_adult_business_02/female_adult_business_02/ManRoot/female_adult_business_02")

            # This should not be None !!!!
            print(character_graph)

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
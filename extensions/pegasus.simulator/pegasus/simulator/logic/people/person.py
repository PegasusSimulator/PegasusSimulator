"""
| File: person.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Person class which is used as the base for spawning people in the simulation world.
"""

# Low level APIs
import carb

# High level Isaac sim APIs
import omni.client
from omni.anim.people import PeopleSettings
from omni.isaac.core.utils import prims
from omni.usd import get_stage_next_free_path
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Extension APIs
from pegasus.simulator.logic.people_manager import PeopleManager
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

class Person:

    # Get root assets path from setting, if not set, get the Isaac-Sim asset path
    setting_dict = carb.settings.get_settings()
    people_asset_folder = setting_dict.get(PeopleSettings.CHARACTER_ASSETS_PATH)
    character_root_prim_path = setting_dict.get(PeopleSettings.CHARACTER_PRIM_PATH)
    assets_root_path = None

    if not character_root_prim_path:
        character_root_prim_path = "/World/Characters"

    if people_asset_folder:
        assets_root_path = people_asset_folder
    else:   
        root_path = get_assets_root_path()
        if root_path is not None:
            assets_root_path  = "{}/Isaac/People/Characters".format(root_path)

    def __init__(
        self, 
        stage_prefix: str,
        usd_path: str = None,
        init_pos=[0.0, 0.0, 0.0],
        init_orientation=[0.0, 0.0, 0.0, 1.0]
    ):

        # Get the current world at which we want to spawn the vehicle
        self._world = PegasusInterface().world
        self._current_stage = self._world.stage

        # Save the name with which the vehicle will appear in the stage
        # and the name of the .usd file that contains its description
        self._stage_prefix = get_stage_next_free_path(self._current_stage, stage_prefix, False)
        self._usd_file = usd_path

        # If there is no XForm primitive in the stage to hold all the people, create one
        if not self._current_stage.GetPrimAtPath(Person.character_root_prim_path):
            prims.create_prim(Person.character_root_prim_path, "Xform")

        # Add the current person to the person manager
        PeopleManager.get_people_manager().add_person(self._stage_prefix, self)

    @staticmethod
    def get_character_asset_list():
        # List all files in characters directory
        result, folder_list = omni.client.list("{}/".format(Person.assets_root_path))

        if result != omni.client.Result.OK:
            carb.log_error("Unable to get character assets from provided asset root path.")
            return

        # Prune items from folder list that are not directories.
        pruned_folder_list = [folder.relative_path for folder in folder_list 
            if (folder.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN) and not folder.relative_path.startswith(".")]

        return pruned_folder_list
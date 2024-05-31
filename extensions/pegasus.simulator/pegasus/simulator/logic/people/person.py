"""
| File: person.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Person class which is used as the base for spawning people in the simulation world.
"""

# Low level APIs
import carb
from pxr import Gf, Sdf

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
        character_name: str = None,
        init_pos=[0.0, 0.0, 0.0],
        init_yaw=0.0
    ):

        # Get the current world at which we want to spawn the vehicle
        self._world = PegasusInterface().world
        self._current_stage = self._world.stage

        # Save the name with which the vehicle will appear in the stage
        # and the character model that will be loaded into the simulator
        self._stage_prefix = get_stage_next_free_path(self._current_stage, Person.character_root_prim_path + '/' + stage_prefix, False)

        # The name of the character in the USD file
        self._character_name = character_name

        # If there is no XForm primitive in the stage to hold all the people, create one
        if not self._current_stage.GetPrimAtPath(Person.character_root_prim_path):
            prims.create_prim(Person.character_root_prim_path, "Xform")

        # Get the USD file corresponding to the character
        self.char_usd_file = Person.get_path_for_character_prim(character_name)

        # Spawn the agent in the world
        self.spawn_agent(self.char_usd_file, self._stage_prefix, init_pos, init_yaw)

        # Add the animation graph to the agent, such that it can move around
        self.add_animation_graph_to_agent()
        

    def spawn_agent(self, usd_file, stage_name, init_pos, init_yaw):

        # Spawn the person in the world
        self.prim = prims.create_prim(stage_name, "Xform", usd_path=usd_file)

        # Set the initial position and orientation of the person
        self.prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(float(init_pos[0]),float(init_pos[1]), float(init_pos[2])))

        if type(self.prim.GetAttribute("xformOp:orient").Get()) == Gf.Quatf:
            self.prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1), float(init_yaw)).GetQuat()))
        else:
            self.prim.GetAttribute("xformOp:orient").Set(Gf.Rotation(Gf.Vec3d(0,0,1), float(init_yaw)).GetQuat())
        
        # Add the current person to the person manager
        PeopleManager.get_people_manager().add_person(self._stage_prefix, self)

    def add_animation_graph_to_agent(self):
        
        # Get the animation graph that we are going to add to the person
        animation_graph = self._current_stage.GetPrimAtPath(Person.character_root_prim_path + "/Biped_Setup/CharacterAnimation/AnimationGraph")

        # Get the Skeleton root of the character
        character_skel_root = Person._transverse_prim(self._current_stage, self._stage_prefix)
        
        # Remove the animation graph attribute if it exists
        omni.kit.commands.execute("RemoveAnimationGraphAPICommand", paths=[Sdf.Path(character_skel_root.GetPrimPath())])
        
        # Add the animation graph and script to the character
        omni.kit.commands.execute("ApplyAnimationGraphAPICommand", paths=[Sdf.Path(character_skel_root.GetPrimPath())], animation_graph_path=Sdf.Path(animation_graph.GetPrimPath()))
        omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[Sdf.Path(character_skel_root.GetPrimPath())])


    @staticmethod
    def _transverse_prim(stage, stage_prefix):

        # Check if the prim is the one we are looking for
        prim = stage.GetPrimAtPath(stage_prefix)

        # If the prim is the one we are looking for, return it
        if prim.GetTypeName() == "SkelRoot":
            return prim

        # Otherwise, get all the children of the prim and keep transversing until we find the SkelRoot
        children = prim.GetAllChildren()

        # If there are no children, return
        if not children or len(children) == 0:
            return None
        
        # Recursively look through the children to get the SkelRoot
        for child in children:
            print("Looking for child")
            print(stage_prefix + "/" + child.GetName())
            prim_child = Person._transverse_prim(stage, stage_prefix + "/" + child.GetName())

            if prim_child is not None:
                return prim_child
            
        return None


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
    
    @staticmethod
    def get_path_for_character_prim(agent_name):

        # Check if a folder with agent_name exists. If exists we load the character, else we load a random character
        agent_folder = "{}/{}".format(Person.assets_root_path, agent_name)
        result, properties = omni.client.stat(agent_folder)

        # Attempt to load the character if it exists, otherwise load a random character
        if result != omni.client.Result.OK:
            carb.log_error("Character folder does not exist.")
            return None
        
        # Get the usd present in the character folder
        character_folder = "{}/{}".format(Person.assets_root_path, agent_name)
        character_usd = Person.get_usd_in_folder(character_folder)
    
        # Return the character name (folder name) and the usd path to the character
        return "{}/{}".format(character_folder, character_usd)
    
    @staticmethod
    def get_usd_in_folder(character_folder_path):
        result, folder_list = omni.client.list(character_folder_path)
        
        if result != omni.client.Result.OK:
            carb.log_error("Unable to read character folder path at {}".format(character_folder_path))
            return

        for item in folder_list:
            if item.relative_path.endswith(".usd"):
                return item.relative_path

        carb.log_error("Unable to file a .usd file in {} character folder".format(character_folder_path))

    
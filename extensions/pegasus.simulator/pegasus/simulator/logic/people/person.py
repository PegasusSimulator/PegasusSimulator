"""
| File: person.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Person class which is used as the base for spawning people in the simulation world.
"""

import numpy as np
from scipy.spatial.transform import Rotation

# Low level APIs
import carb
from pxr import Sdf

# High level Isaac sim APIs
import NavSchema
import omni.client
from omni.usd import get_stage_next_free_path
from isaacsim.storage.native import get_assets_root_path

# New imports from the replicator API
import omni.anim.graph.core as ag
import isaacsim.replicator.agent.core
from isaacsim.replicator.agent.core.settings import PrimPaths
from isaacsim.replicator.agent.core.stage_util import CharacterUtil
from isaacsim.replicator.agent.core.simulation import SimulationManager

# Extension APIs
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.people_manager import PeopleManager
from pegasus.simulator.logic.people.person_controller import PersonController
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

class Person:
    """
    Class that implements a person in the simulation world. The person can be controlled by a controller that inherits from the PersonController class.
    """

    # Get root assets path from setting, if not set, get the Isaac-Sim asset path
    people_asset_folder = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/People/Characters/"
    character_root_prim_path = PrimPaths.characters_parent_path()

    assets_root_path = None   

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
        init_yaw=0.0,
        controller: PersonController=None,
        backend=None
    ):
        """Initializes the person object

        Args:
            stage_prefix (str): The name the person will present in the simulator when spawned on the stage.
            character_name (str): The name of the person in the USD file. Use the Person.get_character_asset_list() method to get the list of available characters.
            init_pos (list): The initial position of the vehicle in the inertial frame (in ENU convention). Defaults to [0.0, 0.0, 0.0].
            init_yaw (float): The initial orientation of the person in rad. Defaults to 0.0.
            controller (PersonController): A controller to add some custom behaviour to the movement of the person. Defaults to None.
        """

        # Get the current world at which we want to spawn the vehicle
        self._world = PegasusInterface().world
        self._current_stage = self._world.stage

        # Load the simulation manager
        self.sim_manager = SimulationManager()

        # Variable that will hold the current state of the vehicle
        self._state = State()
        self._state.position = np.array(init_pos)
        self._state.orientation = Rotation.from_euler('z', init_yaw, degrees=False).as_quat()

        # Set the target position for the character
        self._target_position = np.array(init_pos)
        self._target_speed = 0.0

        # By default, the characters are placed inside /World/Characters
        # so we are checking whether the name of the character is already present in the stage inside the character root prim path
        # or we need to change the name of the character to avoid conflicts
        self._stage_prefix = get_stage_next_free_path(self._current_stage, Person.character_root_prim_path + '/' + stage_prefix, False)

        # The name of the character in the USD file
        self._character_name = character_name

        # Get the USD file corresponding to the character
        self.char_usd_file = Person.get_path_for_character_prim(character_name)

        # Spawn the agent in the world
        self.spawn_agent(self.char_usd_file, self._stage_prefix, init_pos, init_yaw)

        # Add the animation graph to the agent, such that it can move around
        self.character_graph = None
        self.add_animation_graph_to_agent()

        # Set the controller for the person if any and initialize it
        self._controller = controller
        if self._controller:
           self._controller.initialize(self)

        # Set the backend for publishing the state of the person
        self._backend = backend
        if self._backend:
            self._backend.initialize(self)

        # Add a callback to the physics engine to update the current state of the person
        self._world.add_physics_callback(self._stage_prefix + "/state", self.update_state)

        # Add the update method to the physics callback if the world was received
        # so that we can apply the new references to be tracked by the person
        self._world.add_physics_callback(self._stage_prefix + "/update", self.update)

        # Set the flag that signals if the simulation is running or not
        self._sim_running = False

        # Add a callback to start/stop of the simulation once the play/stop button is hit
        self._world.add_timeline_callback(self._stage_prefix + "/start_stop_sim", self.sim_start_stop)

    @property
    def state(self):
        """The state of the person.

        Returns:
            State: The current state of the person, i.e., position, orientation, linear and angular velocities...
        """
        return self._state
    
    def sim_start_stop(self, event):
        """
        Callback that is called every time there is a timeline event such as starting/stoping the simulation.

        Args:
            event: A timeline event generated from Isaac Sim, such as starting or stoping the simulation.
        """

        # If the start/stop button was pressed, then call the start and stop methods accordingly
        if self._world.is_playing() and self._sim_running == False:
            self._sim_running = True
            self.start()

        if self._world.is_stopped() and self._sim_running == True:
            self._sim_running = False
            self.stop()

    def start(self):
        """
        Method that is called when the simulation starts. This method can be used to initialize any variables.
        """
        if self._controller:
            self._controller.start()

    def stop(self):
        """
        Method that is called when the simulation stops. This method can be used to reset any variables.
        """
        if self._controller:
            self._controller.stop()

    def update(self, dt: float):
        """
        Method that implements the logic to make the person move around in the simulation world and also play the animation

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Note: this is done to avoid the error of the character_graph being None. The animation graph is only created after the simulation starts
        if not self.character_graph or self.character_graph is None:
            self.character_graph = ag.get_character(self.character_skel_root_stage_path)

        # If the character graph is not None, then we can update the character
        if self.character_graph:

            # Call the controller update method that should update the reference of the target position
            if self._controller:
                self._controller.update(dt)

            # Compute the distance between the current position and the goal position
            distance_to_target_position = np.linalg.norm(self._target_position - self._state.position)
            
            # If we are still far away from the target position, keep moving towards it
            if distance_to_target_position > 0.1:
                self.character_graph.set_variable("Action", "Walk")
                self.character_graph.set_variable("PathPoints", [carb.Float3(self._state.position), carb.Float3(self._target_position)])
                self.character_graph.set_variable("Walk", self._target_speed)
            else:
                # If we are close to the target position, stop moving
                self.character_graph.set_variable("Walk", 0.0)
                self.character_graph.set_variable("Action", "Idle")

            # If we have a backend, update the state of the person
            if self._backend:
                self._backend.update(self._state, dt)


    def update_target_position(self, position, walk_speed=1.0):
        """
        Method that updates the target position of the person to which it will move towards.

        Args:
            position (list): A list with the x, y, z coordinates of the target position.
        """
        self._target_position = np.array(position)
        self._target_speed = walk_speed


    def update_state(self, dt: float):
        """
        Method that is called at every physics step to retrieve and update the current state of the person, i.e., get
        the current position, orientation, linear and angular velocities and acceleration of the person.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        
        # # Note: this is done to avoid the error of the character_graph being None. The animation graph is only created after the simulation starts
        if not self.character_graph or self.character_graph is None:
            self.character_graph = ag.get_character(self.character_skel_root_stage_path)
            
        # If the character graph is not None, then we can update the character
        if self.character_graph:

            # Get the current position of the person
            pos = carb.Float3(0, 0, 0)
            rot = carb.Float4(0, 0, 0, 0)
            self.character_graph.get_world_transform(pos, rot)

            # Update the current state of the person
            self._state.position = np.array([pos[0], pos[1], pos[2]])
            self._state.orientation = np.array([rot.x, rot.y, rot.z, rot.w])

            # Signal the controller the updated state
            if self._controller:
                self._controller.update_state(self._state)


    def spawn_agent(self, usd_file, stage_name, init_pos, init_yaw):

        # Get the last name after the last slash in the stage name
        # We only want the name of the character, not the full path
        character_name = stage_name.split("/")[-1]

        # Spawn the character in the stage using the NVIDIA replicar API (which is suprisingly similar to the original version I provided in this function in version 4.2.0)
        CharacterUtil.load_character_usd_to_stage(usd_file, init_pos, init_yaw, character_name)

        # Add the current person to the person manager
        PeopleManager.get_people_manager().add_person(self._stage_prefix, self)

        # Get the handle to the character skeleton root prim
        self.character_skel_root, self.character_skel_root_stage_path = Person._transverse_prim(self._current_stage, self._stage_prefix)

        # Update the navigation mesh to include the character skeleton root prim
        omni.kit.commands.execute("ApplyNavMeshAPICommand", prim_path=stage_name, api=NavSchema.NavMeshExcludeAPI)

    def add_animation_graph_to_agent(self):
        
        # Load the character skeleton and animations (if not loaded yet)
        self.sim_manager.load_default_skeleton_and_animations()

        # The part bellow is based on "setup_animation_graph_to_character" from the replicator SimulationManager class
        default_biped_prim = PrimPaths.biped_prim_path()
        anim_graph_prim = CharacterUtil.get_anim_graph_from_character(self._current_stage.GetPrimAtPath(default_biped_prim))
        self.sim_manager.setup_animation_graph_to_character([self.character_skel_root])

        # Remove the animation graph attribute if it exists
        omni.kit.commands.execute("RemoveAnimationGraphAPICommand", paths=[Sdf.Path(self.character_skel_root.GetPrimPath())])

        # Add the animation graph to the character
        omni.kit.commands.execute("ApplyAnimationGraphAPICommand", paths=[Sdf.Path(self.character_skel_root.GetPrimPath())], animation_graph_path=Sdf.Path(anim_graph_prim.GetPrimPath()))

    @staticmethod
    def _transverse_prim(stage, stage_prefix):

        # Check if the prim is the one we are looking for
        prim = stage.GetPrimAtPath(stage_prefix)

        # If the prim is the one we are looking for, return it
        if prim.GetTypeName() == "SkelRoot":
            return prim, stage_prefix

        # Otherwise, get all the children of the prim and keep transversing until we find the SkelRoot
        children = prim.GetAllChildren()

        # If there are no children, return
        if not children or len(children) == 0:
            return None, None
        
        # Recursively look through the children to get the SkelRoot
        for child in children:
            prim_child, child_stage_prefix = Person._transverse_prim(stage, stage_prefix + "/" + child.GetName())

            if prim_child is not None:
                return prim_child, child_stage_prefix
            
        return None, None


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

    
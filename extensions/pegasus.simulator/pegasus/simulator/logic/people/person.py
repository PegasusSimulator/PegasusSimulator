"""
| File: person.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Person class which is used as the base for spawning people in the simulation world.
"""

import asyncio
import numpy as np
from scipy.spatial.transform import Rotation

# Low level APIs
import carb
from pxr import Sdf, Gf, UsdGeom

# High level Isaac sim APIs
import NavSchema
import omni.client
import omni.kit.app
import omni.usd
from omni.usd import get_stage_next_free_path
from isaacsim.storage.native import get_assets_root_path
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent

# omni.anim.graph.core is in extscache in Isaac Sim 6.0 and must be enabled before import
from isaacsim.core.experimental.utils.app import enable_extension
enable_extension("omni.anim.graph.core")
import omni.anim.graph.core as ag

# Extension APIs
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.people_manager import PeopleManager
from pegasus.simulator.logic.people.person_controller import PersonController
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface


class Person:
    """
    Class that implements a person in the simulation world. The person can be controlled by a controller that inherits from the PersonController class.
    """

    # Can be set by users to override the default NVIDIA asset server URL.
    # When None, the path is resolved at runtime via get_assets_root_path() so
    # the correct Nucleus/S3 URL for this Isaac Sim installation is used.
    people_asset_folder = None

    # Default parent path for character prims (matches isaacsim.replicator.agent.core config default)
    character_root_prim_path = "/World/Characters"

    # Resolved lazily on first Person instantiation (not at module import time)
    assets_root_path = None

    @classmethod
    def _ensure_assets_root_path(cls):
        """Resolve assets_root_path on first call. Called from __init__ and static helpers."""
        if cls.assets_root_path is not None:
            return
        if cls.people_asset_folder:
            cls.assets_root_path = cls.people_asset_folder
        else:
            try:
                root_path = get_assets_root_path()
                if root_path:
                    cls.assets_root_path = f"{root_path}/Isaac/People/Characters"
            except Exception as e:
                carb.log_error(f"Could not resolve Isaac Sim asset root path: {e}")

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

        # Get the current stage
        self._current_stage = omni.usd.get_context().get_stage()

        # Ensure the assets root path is resolved before first use
        Person._ensure_assets_root_path()

        # Variable that will hold the current state of the vehicle
        self._state = State()
        self._state.position = np.array(init_pos)
        self._state.attitude = Rotation.from_euler('z', init_yaw, degrees=False).as_quat()

        # Auxiliar variable to compute the velocity of the person using discrete differentiation
        self._previous_position = np.array(init_pos)
        self._total_dt = 0.0

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

        # Characters assets path:
        # https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/People/Characters/

        # Add the animation graph to the agent, such that it can move around.
        # ApplyBehaviorAgentAPICommand triggers OmniGraph creation which segfaults when
        # called synchronously before the first app update; schedule it async instead.
        self.character_graph = None
        asyncio.ensure_future(self._setup_animation_graph_async())

        # Set the controller for the person if any and initialize it
        self._controller = controller
        if self._controller:
           self._controller.initialize(self)

        # Set the backend for publishing the state of the person
        self._backend = backend
        if self._backend:
            self._backend.initialize(self)

        # Register physics callbacks with SimulationManager
        self._cb_state = SimulationManager.register_callback(self.update_state, SimulationEvent.PHYSICS_POST_STEP)
        self._cb_update = SimulationManager.register_callback(self.update, SimulationEvent.PHYSICS_POST_STEP)

        # Set the flag that signals if the simulation is running or not
        self._sim_running = False

        # Register simulation start/stop callbacks
        self._cb_sim_start = SimulationManager.register_callback(self._on_sim_started, SimulationEvent.SIMULATION_STARTED)
        self._cb_sim_stop = SimulationManager.register_callback(self._on_sim_stopped, SimulationEvent.SIMULATION_STOPPED)

    def __del__(self):
        for cb_id in (self._cb_state, self._cb_update, self._cb_sim_start, self._cb_sim_stop):
            try:
                SimulationManager.deregister_callback(cb_id)
            except Exception:
                pass

    @property
    def state(self):
        """The state of the person.

        Returns:
            State: The current state of the person, i.e., position, orientation, linear and angular velocities...
        """
        return self._state
    
    def _on_sim_started(self, *args):
        """Callback invoked when the simulation starts."""
        if not self._sim_running:
            self._sim_running = True
            self.start()

    def _on_sim_stopped(self, *args):
        """Callback invoked when the simulation stops."""
        if self._sim_running:
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

    def update(self, dt: float, context=None):
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


    def update_state(self, dt: float, context=None):
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
            self._state.attitude = np.array([rot.x, rot.y, rot.z, rot.w])

            # Compute the velocity in the inertial frame using discrete differentiation
            self._total_dt += dt

            # Note: we are updating the linear velocity at 20 Hz to avoid noise in the velocity estimation
            # this is not ideal, but it works for now until NVIDIA provides a better way to get the linear velocity of the character
            if self._total_dt > 1/20.0:  # Update at 20 Hz
                self._state.linear_velocity = (self._state.position - self._previous_position) / self._total_dt
                self._previous_position = np.array(self._state.position)
                self._total_dt = 0.0

            # Signal the controller the updated state
            if self._controller:
                self._controller.update_state(self._state)


    def spawn_agent(self, usd_file, stage_name, init_pos, init_yaw):

        # CharacterUtil.load_character_usd_to_stage was removed in Isaac Sim 6.0.
        # Replicate its behavior: add a USD reference at the stage path and set the initial xform.
        stage_utils.add_reference_to_stage(usd_path=usd_file, path=stage_name)
        prim = self._current_stage.GetPrimAtPath(stage_name)
        xformable = UsdGeom.Xformable(prim)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3f(float(init_pos[0]), float(init_pos[1]), float(init_pos[2])))
        yaw_quat = Rotation.from_euler('z', init_yaw, degrees=False).as_quat()  # [qx, qy, qz, qw]
        xformable.AddOrientOp().Set(Gf.Quatf(float(yaw_quat[3]), float(yaw_quat[0]), float(yaw_quat[1]), float(yaw_quat[2])))

        # Add the current person to the person manager
        PeopleManager.get_people_manager().add_person(self._stage_prefix, self)

        # Get the handle to the character skeleton root prim
        self.character_skel_root, self.character_skel_root_stage_path = Person._transverse_prim(self._current_stage, self._stage_prefix)

        # Update the navigation mesh to include the character skeleton root prim
        omni.kit.commands.execute("ApplyNavMeshAPICommand", prim_path=stage_name, api=NavSchema.NavMeshExcludeAPI)

        # Load HumanMotionLibrary (replaces Biped_Setup.usd in Isaac Sim 6.0)
        motion_library_prim_path = Person.character_root_prim_path + "/HumanMotionLibrary"
        if not self._current_stage.GetPrimAtPath(motion_library_prim_path):
            try:
                root_path = get_assets_root_path()
                motion_library_url = f"{root_path}/Isaac/People/MotionLibrary/HumanMotionLibrary.usd"
                omni.kit.commands.execute(
                    "CreatePayloadCommand",
                    usd_context=omni.usd.get_context(),
                    path_to=motion_library_prim_path,
                    asset_path=motion_library_url,
                    prim_path=None,
                    instanceable=False,
                    select_prim=False,
                )
            except Exception as e:
                carb.log_error(f"Failed to load HumanMotionLibrary: {e}")


    async def _setup_animation_graph_async(self):
        # Wait one frame so the HumanMotionLibrary payload is fully resolved and
        # OmniGraph is ready to accept new nodes (avoids segfault in standalone mode).
        await omni.kit.app.get_app().next_update_async()
        self.add_animation_graph_to_agent()

    def add_animation_graph_to_agent(self):
        # In Isaac Sim 6.0, ApplyBehaviorAgentAPICommand replaces AnimationGraphAPI + Biped_Setup.usd.
        # Apply the BehaviorAgentAPI to the character's SkelRoot, pointing at the HumanMotionLibrary.
        if self.character_skel_root is None:
            carb.log_warn("No SkelRoot found for character — skipping BehaviorAgentAPI setup.")
            return
        motion_library_prim_path = Person.character_root_prim_path + "/HumanMotionLibrary"
        omni.kit.commands.execute(
            "ApplyBehaviorAgentAPICommand",
            skelroot_prim_paths=[self.character_skel_root.GetPath()],
            motion_library_prim_path=motion_library_prim_path,
            motion_library_skeleton_rig="Human",
        )


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
        Person._ensure_assets_root_path()
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

    
#!/usr/bin/env python

# Numerical computations
import numpy as np
from scipy.spatial.transform import Rotation

# Low level APIs
import carb
from pxr import Usd, Gf

# High level Isaac sim APIs
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim
from omni.usd import get_stage_next_free_path
from omni.isaac.core.robots.robot import Robot

# Extension APIs
from pegasus_isaac.logic.state import State
from pegasus_isaac.logic.vehicles.vehicle_manager import VehicleManager


def get_world_transform_xform(prim: Usd.Prim):
    """
    Get the local transformation of a prim using omni.usd.get_world_transform_matrix().
    See https://docs.omniverse.nvidia.com/kit/docs/omni.usd/latest/omni.usd/omni.usd.get_world_transform_matrix.html
    Args:
        prim: The prim to calculate the world transformation.
    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
        - Scale vector.
    """
    world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
    rotation: Gf.Rotation = world_transform.ExtractRotation()
    return rotation


class Vehicle(Robot):

    def __init__(
        self, 
        stage_prefix: str, 
        usd_path: str = None, 
        world: World=None, 
        init_pos=[0.0, 0.0, 0.0], 
        init_orientation=[0.0, 0.0, 0.0, 1.0]
    ):
        """
        Class that initializes a vehicle in the isaac sim's curent stage
        """

        # Get the current world at which we want to spawn the vehicle
        self._world = world
        self._current_stage = world.stage

        # Save the name with which the vehicle will appear in the stage
        # and the name of the .usd file that contains its description
        self._stage_prefix = get_stage_next_free_path(self._current_stage, stage_prefix, False)
        self._usd_file = usd_path
        
        # # Spawn the vehicle primitive in the world's stage
        self._prim = define_prim(self._stage_prefix, "Xform")
        self._prim.GetReferences().AddReference(self._usd_file)

        # # Initialize the "Robot" class
        # # Note: we need to change the rotation to have qw first, because NVidia
        # # does not keep a standard of quaternions inside its own libraries (not good, but okay)
        super().__init__(
           prim_path=self._stage_prefix, 
           name=self._stage_prefix, 
           position=init_pos, 
           orientation=[init_orientation[3], init_orientation[0], init_orientation[1], init_orientation[2]], 
           articulation_controller=None
        )

        # Add this object for the world to track, so that if we clear the world, this object is deleted from memory and
        # as a consequence, from the VehicleManager as well
        self._world.scene.add(self)

        # Add the current vehicle to the vehicle manager, so that it knows
        # that a vehicle was instantiated
        VehicleManager.get_vehicle_manager().add_vehicle(self._stage_prefix, self)

        # Variable that will hold the current state of the vehicle
        self._state = State()

        # Motor that is given as reference
        self._motor_speed = []

        # Add a callback to the physics engine to update the current state of the system
        self._world.add_physics_callback(self._stage_prefix + "/state", self.update_state)

        # Add the update method to the physics callback if the world was received
        # so that we can apply forces and torques to the vehicle. Note, this method should
        # be implemented in classes that inherit the vehicle object
        self._world.add_physics_callback(self._stage_prefix + "/update", self.update)

        # Set the flag that signals if the simulation is running or not
        self._sim_running = False

        # Add a callback to start/stop of the simulation once the play/stop button is hit
        self._world.add_timeline_callback(self._stage_prefix + "/start_stop_sim", self.sim_start_stop)

    def __del__(self):

        # Remove this object from the vehicleHandler
        VehicleManager.get_vehicle_manager().remove_vehicle(self._stage_prefix)

    """
    Properties
    """

    @property
    def state(self):
        return self._state

    @property
    def motor_speed(self):
        return self._motor_speed

    """
    Operations
    """

    def sim_start_stop(self, event):
        """
        Callback that is called every time there is a timeline event such as starting/stoping the simulation
        """
        
        # If the start/stop button was pressed, then call the start and stop methods accordingly
        if self._world.is_playing() and self._sim_running == False:
            self._sim_running = True
            self.start()

        if self._world.is_stopped() and self._sim_running == True:
            self._sim_running = False
            self.stop()

    def apply_force(self, force, pos=[0.0, 0.0, 0.0], body_part="/body"):
        """
        Method that when invoked applies a given force vector to the rigid body or /<rigid_body_name>/"body"
        """

        # Get the handle of the rigidbody that we will apply the force to
        rb = self._world.dc_interface.get_rigid_body(self._stage_prefix  + "/vehicle" + body_part)

        # Apply the force to the rigidbody. The force should be expressed in the rigidbody frame
        self._world.dc_interface.apply_body_force(rb, carb._carb.Float3(force), carb._carb.Float3(pos), False)

    def apply_torque(self, torque, body_part=""):
        """
        Method that when invoked applies a given torque vector to the rigid body or /<rigid_body_name>/"body"
        """
        
        # Get the handle of the rigidbody that we will apply a torque to
        rb = self._world.dc_interface.get_rigid_body(self._stage_prefix  + "/vehicle" + body_part)

        # Apply the torque to the rigidbody. The torque should be expressed in the rigidbody frame
        self._world.dc_interface.apply_body_torque(rb, carb._carb.Float3(torque), False)

    def update_state(self, dt: float):

        # Get the body frame interface of the vehicle (this will be the frame used to get the position, orientation, etc.)
        body = self._world.dc_interface.get_rigid_body(self._stage_prefix + "/vehicle/body")

        # Get the current position and orientation in the inertial frame
        pose = self._world.dc_interface.get_rigid_body_pose(body)

        # NOTE: the attitude given by get_rigid_body_pose is not updating correctly (I guess this is a bug on the NVidia API)
        # Now, here is the question: is it a bug or is it a feature? Let me know your opinion - I'm curious XD
        # But seriously, NVidia, if you are reading this, please fix it and let me know. Robotics people like me do not expect
        # this behaviour from the get_rigid_body_pose method. It is only giving the me initial orientation the vehicle was spawned with
        # Get the attitude according to the convention [w, x, y, z] using the internal Pixar library instead
        prim = self._world.stage.GetPrimAtPath(self._stage_prefix + "/vehicle/body")
        rotation_quat = get_world_transform_xform(prim).GetQuaternion()
        rotation_quat_real = rotation_quat.GetReal()
        rotation_quat_img = rotation_quat.GetImaginary()

        # Get the angular velocity of the vehicle expressed in the body frame of reference
        ang_vel = self._world.dc_interface.get_rigid_body_angular_velocity(body)

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        linear_vel = self._world.dc_interface.get_rigid_body_linear_velocity(body)

        # Get the linear acceleration of the body relative to the inertial frame, expressed in the inertial frame
        # Note: we must do this approximation, since the Isaac sim does not output the acceleration of the rigid body directly
        linear_acceleration = (np.array(linear_vel) - self._state.linear_velocity) / dt

        # Update the state variable X = [x,y,z]
        self._state.position = np.array(pose.p)

        # Get the quaternion according in the [qx,qy,qz,qw] standard
        self._state.attitude = np.array([rotation_quat_img[0], rotation_quat_img[1], rotation_quat_img[2], rotation_quat_real])

        # Express the velocity of the vehicle in the inertial frame X_dot = [x_dot, y_dot, z_dot]
        self._state.linear_velocity = np.array(linear_vel)

        # The linear velocity V =[u,v,w] of the vehicle's body frame expressed in the body frame of reference
        # Note that: x_dot = Rot * V
        self._state.linear_body_velocity = Rotation.from_quat(self._state.attitude).inv().apply(self._state.linear_velocity)
        
        # NOTE: for some reason, NVIDIA gives the angular velocity in a reference frame that is aligned with
        # the inertial frame, instead of the vehicle body frame, so we need to manually rotate it.. ARrrr
        # NVidia, let me know when you fix this. I spent way too many hours debugging, because of this. You can do better!
        # omega = [p,q,r]
        self._state.angular_velocity = Rotation.from_quat(self._state.attitude).inv().apply(np.array(ang_vel))
        
        # The acceleration of the vehicle expressed in the inertial frame X_ddot = [x_ddot, y_ddot, z_ddot]
        self._state.linear_acceleration = linear_acceleration

    def start(self):
        """
        Method that should be implemented by the class that inherits the vehicle object.
        """
        pass

    def stop(self):
        """
        Method that should be implemented by the class that inherits the vehicle object.
        """
        pass

    def update(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type and it's called periodically by the physics engine
        """
        pass

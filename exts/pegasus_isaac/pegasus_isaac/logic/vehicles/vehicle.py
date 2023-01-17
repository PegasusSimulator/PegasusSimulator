#!/usr/bin/env python

import numpy as np

import carb
from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim
from omni.physx import get_physx_interface
from omni.usd import get_stage_next_free_path
from omni.isaac.core.robots.robot import Robot

from pegasus_isaac.logic.state import State
from pegasus_isaac.logic.vehicles.vehicle_manager import VehicleManager

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
        
        # Spawn the vehicle primitive in the world's stage
        self._prim = define_prim(self._stage_prefix, "Xform")
        self._prim.GetReferences().AddReference(self._usd_file)

        # Initialize the "Robot" class
        # Note: we need to change the rotation to have qw first, because NVidia
        # does not keep a standard of quaternions inside its own libraries (not good, but okay)
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

        # Get a physics interface, so that we can apply forces and torques directly to the rigid body
        self._physx_interface = get_physx_interface()

        # Add the current vehicle to the vehicle manager, so that it knows
        # that a vehicle was instantiated
        VehicleManager.get_vehicle_manager().add_vehicle(self._stage_prefix, self)

        # Variable that will hold the current state of the vehicle
        self._state = State()

        # Motor that is given as reference
        self._motor_speed = []

        # Add a callback to the physics engine to update the current state of the system
        self._world.add_physics_callback(self._stage_prefix + "/state", self.update_current_state)

        # Add the apply_forces method to the physics callback if the world was received
        # so that we can apply forces and torques to the vehicle. Note, this method should
        # be implemented in classes that inherit the vehicle object
        self._world.add_physics_callback(self._stage_prefix + "/apply_forces", self.apply_forces)

    def __del__(self):

        # Remove the physics callback so that we don't get a segmentation fault
        try:
            self._world.remove_physics_callback(self._stage_prefix + "/state")
            self._world.remove_physics_callback(self._stage_prefix + "/apply_forces")
        except:
            carb.log_info("Physics callbacks were already cleaned")

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

    def apply_force(self, force, pos=[0.0, 0.0, 0.0], body_part=""):
        """
        Method that when called, applies a given force vector to the rigid body or /<rigid_body_name>/"body"
        specified.
        """
        #carb.log_info(self._stage_prefix + body_part + "  " + str(force))

        self._physx_interface.apply_force_at_pos(
            self._stage_prefix + body_part, carb._carb.Float3(force), carb._carb.Float3(pos))

    def update_current_state(self, dt):

        # Get the body frame interface of the vehicle (this will be the frame used to get the position, orientation, etc.)
        body = self._world.dc_interface.get_rigid_body(self._stage_prefix  + "/vehicle/body")

        # Get the current position and orientation in the inertial frame
        pose = self._world.dc_interface.get_rigid_body_pose(body)

        # NOTE: the attitude given by get_rigid_body_pose is not updating correctly (I guess this is a bug on the NVidia API)
        # Now, here is the question: is it a bug or is it a feature? Let me know your opinion - I'm curious XD
        # But seriously, NVidia, if you are reading this, please fix it and let me know. Robotics people like me do not expect
        # this behaviour from the get_rigid_body_pose method. It is only giving the me initial orientation the vehicle was spawned with
        #carb.log_warn((self._world.stage.GetPrimAtPath(self._stage_prefix  + "/vehicle/body")))

        #for att in self._world.stage.GetPrimAtPath(self._stage_prefix  + "/vehicle/body").GetAttributes():
        #    carb.log_warn(att)

        carb.log_warn(self._world.stage.GetPrimAtPath(self._stage_prefix  + "/vehicle/body").GetAttribute('xformOp:orient'))
        carb.log_warn("-------------")

        # Get the angular velocity of the vehicle expressed in the body frame of reference
        ang_vel = self._world.dc_interface.get_rigid_body_angular_velocity(body)

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        linear_vel = self._world.dc_interface.get_rigid_body_linear_velocity(body)

        # The linear velocity [u,v,w] of the vehicle's body frame expressed in the body frame of reference
        linear_vel_body = self._world.dc_interface.get_rigid_body_local_linear_velocity(body)

        carb.log_warn(linear_vel_body)

        # Get the linear acceleration of the body relative to the inertial frame, expressed in the inertial frame
        # Note: we must do this approximation, since the Isaac sim does not output the acceleration of the rigid body directly
        # TODO - check if this is being computed correctly
        linear_acceleration = (self._state.linear_velocity - np.array(linear_vel)) / dt

        # Update the state variable
        self._state.position = np.array(pose.p)
        self._state.attitude = np.array(pose.r)

        self._state.linear_body_velocity = np.array(linear_vel_body)
        self._state.linear_velocity = np.array(linear_vel)

        self._state.angular_velocity = np.array(ang_vel)
        self._state.linear_acceleration = linear_acceleration

    def apply_forces(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type and it's called periodically by the physics engine
        """
        pass

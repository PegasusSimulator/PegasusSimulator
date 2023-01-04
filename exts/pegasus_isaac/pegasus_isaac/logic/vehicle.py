#!/usr/bin/env python

from omni.usd import get_stage_next_free_path
from omni.isaac.core.utils.stage import get_current_stage

from pegasus_isaac.logic.state import State
from pegasus_isaac.logic.vehicle_manager import VehicleManager

class Vehicle:

    def __init__(self, stage_prefix: str, usd_file: str):
        """
        Class that initializes a vehicle in the isaac sim's curent stage
        """
        
        # Get the current stage at which we want to spawn the vehicle at
        self._current_stage = get_current_stage()

        # Save the name with which the vehicle will appear in the stage
        # and the name of the .usd file that contains its description
        self._stage_prefix = get_stage_next_free_path(self._current_stage, stage_prefix, False)
        self._usd_file = usd_file
        
        # Spawn the vehicle primitive in the world's stage
        self._prim = self._current_stage.DefinePrim(self.stage_prefix, "Xform")
        self._prim.GetReferences().AddReference(self._usd_file)

        # Add the current vehicle to the vehicle manager, so that it knows
        # that a vehicle was instantiated
        VehicleManager.get_vehicle_manager().add_vehicle(self.stage_prefix, self)

        # Variable that will hold the current state of the vehicle
        self._state = State()

        # Motor that is given as reference
        self._motor_speed = []

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

    def update_state(self, state):
        """
        Method that updates the current state of the vehicle
        """
        self._state = state

    def update_motor_speed(self, motor_speed):
        """
        Method that updates the desired motor speed for the vehicle 
        """
        self._motor_speed = motor_speed

    def apply_forces(self):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type
        """
        pass
    
#!/usr/bin/env python

import carb

from pegasus_isaac.logic.vehicle import Vehicle
from pegasus_isaac.logic.sensors import Barometer

class Quadrotor(Vehicle):

    def __init__(
        self, 
        stage_prefix: str="quadrotor",  
        usd_file: str="",
        world=None,
        init_pos=[0.0, 0.0, 1.0], 
        init_orientation=[0.0, 0.0, 0.0, 1.0]
    ):
        
        # Initiate the Vehicle
        super().__init__(stage_prefix, usd_file, world, init_pos, init_orientation)

        # Create the sensors that a quadrotor typically has
        self._barometer = Barometer(init_pos[2])
        #self._imu = IMU()
        #self._magnetometer = Magnetometer()
        #self._gps = GPS()

        # Add callbacks to the physics engine to update the sensors every timestep
        #self._world.add_physics_callback(self._stage_prefix + "/barometer", self.update_barometer_sensor)

    def update_barometer_sensor(self, dt: float):
        self._barometer.update(self._state, dt)

    def apply_forces(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type
        """

        # Try to apply upwards force to the rigid body
        self.apply_force([0.0, 0.0, 30.0], body_part="/body")
        
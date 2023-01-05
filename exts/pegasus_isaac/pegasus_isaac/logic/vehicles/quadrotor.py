#!/usr/bin/env python

import carb

from pegasus_isaac.logic.vehicles.vehicle import Vehicle
from pegasus_isaac.logic.sensors import Barometer, IMU, Magnetometer, GPS
import omni.isaac.core.utils.rotations 

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
        self._imu = IMU()
        # TODO - read the latitude and longitude from the world configuration
        self._magnetometer = Magnetometer(38.765824, -9.092815)
        # TODO - check if we can initialize with relative altitude or we need an absolute one
        self._gps = GPS(38.765824, -9.092815, origin_altitude=init_pos[2])

        # Add callbacks to the physics engine to update the sensors every timestep
        self._world.add_physics_callback(self._stage_prefix + "/barometer", self.update_barometer_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/imu", self.update_imu_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/magnetometer", self.update_magnetometer_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/gps", self.update_gps_sensor)

    def update_barometer_sensor(self, dt: float):
        result = self._barometer.update(self._state, dt)
        #carb.log_warn(result["pressure_altitude"])

    def update_imu_sensor(self, dt: float):
        result = self._imu.update(self._state, dt)
        #carb.log_warn(result)

    def update_magnetometer_sensor(self, dt: float):
        result = self._magnetometer.update(self._state, dt)
        #carb.log_warn(result)

    def update_gps_sensor(self, dt: float):
        result = self._gps.update(self._state, dt)
        carb.log_warn(result)

    def apply_forces(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type
        """

        # Try to apply upwards force to the rigid body
        self.apply_force([0.0, 0.0, 9.82], body_part="/body")
        
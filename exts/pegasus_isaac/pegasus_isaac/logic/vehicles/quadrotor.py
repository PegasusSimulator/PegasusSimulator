#!/usr/bin/env python

import carb

from pegasus_isaac.logic.vehicles.vehicle import Vehicle
from pegasus_isaac.mavlink_interface import MavlinkInterface
from pegasus_isaac.logic.sensors import Barometer, IMU, Magnetometer, GPS
import omni.isaac.core.utils.rotations 

# TODO - remove this - only used for debugging
import threading

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
        self._magnetometer = Magnetometer(38.765824, -9.092815)
        self._gps = GPS(38.765824, -9.092815, origin_altitude=init_pos[2])
        
        # Create a mavlink interface for getting data
        self._mavlink = MavlinkInterface('tcpin:localhost:4560')

        # Add callbacks to the physics engine to update the sensors every timestep
        self._world.add_physics_callback(self._stage_prefix + "/barometer", self.update_barometer_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/imu", self.update_imu_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/magnetometer", self.update_magnetometer_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/gps", self.update_gps_sensor)

        # Add callback for the mavlink communication layer
        #self._world.add_physics_callback(self._stage_prefix + "/mavlink", self.update_mavlink)

    def update_barometer_sensor(self, dt: float):
        carb.log_warn(threading.current_thread().name)
        self._barometer.update(self._state, dt)

    def update_imu_sensor(self, dt: float):
        carb.log_warn(threading.current_thread().name)
        self._imu.update(self._state, dt)

    def update_magnetometer_sensor(self, dt: float):
        carb.log_warn(threading.current_thread().name)
        self._magnetometer.update(self._state, dt)

    def update_gps_sensor(self, dt: float):
        carb.log_warn(threading.current_thread().name)
        self._gps.update(self._state, dt)

    def update_mavlink(self, dt: float):

        # Poll for mavlink msgs (receive the control input for the thrusters)
        self._mavlink.poll_events()

        # Method to send mavlink sensor data from the simulator
        self._mavlink.send_sensors(0, self._imu.state, self._magnetometer.state, self._barometer.state)


    def apply_forces(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type
        """

        # Try to apply upwards force to the rigid body
        self.apply_force([0.0, 0.0, 9.82], body_part="/body")
        
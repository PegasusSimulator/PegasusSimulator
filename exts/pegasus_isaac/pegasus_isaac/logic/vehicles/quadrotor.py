#!/usr/bin/env python

import carb
import numpy as np
from pegasus_isaac.logic.vehicles.vehicle import Vehicle
from pegasus_isaac.mavlink_interface import MavlinkInterface
from pegasus_isaac.logic.sensors import Barometer, IMU, Magnetometer, GPS

import omni.timeline

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
        self._barometer = Barometer(init_pos[2], altitude_home=488.0)   # Check
        self._imu = IMU()                                               # Check
        self._magnetometer = Magnetometer(47.397742, 8.545594)
        self._gps = GPS(47.397742, 8.545594, origin_altitude=488.0)     # Check
        
        # Create a mavlink interface for getting data
        self._mavlink = MavlinkInterface('tcpin:localhost:4560')

        # Add callbacks to the physics engine to update the sensors every timestep
        self._world.add_physics_callback(self._stage_prefix + "/barometer", self.update_barometer_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/imu", self.update_imu_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/magnetometer", self.update_magnetometer_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/gps", self.update_gps_sensor)

        # Add a callback to start/stop the mavlink streaming once the play/stop button is hit
        #self._world.add_timeline_callback(self._stage_prefix + "/start_stop_sim", self.sim_start_stop)

    def update_barometer_sensor(self, dt: float):
        self._barometer.update(self._state, dt)

    def update_imu_sensor(self, dt: float):
        self._imu.update(self._state, dt)
        carb.log_warn("Linear acceleration: " + str(self._imu.state["linear_acceleration"]))

    def update_magnetometer_sensor(self, dt: float):
        self._magnetometer.update(self._state, dt)

    def update_gps_sensor(self, dt: float):
        self._gps.update(self._state, dt)

    def sim_start_stop(self, event):
        """
        Callback that is called every time there is a timeline event such as starting/stoping the simulation
        """
        
        # If the start/stop button was pressed, then start/stop mavlink communication
        if self._world.is_playing():
            self._mavlink.start_stream()
        
        if self._world.is_stopped():
            self._mavlink.stop_stream()

    def apply_forces(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type
        """

        # Try to apply upwards force to the rigid body
        pass
        #self.apply_force([0.0, 0.0, 9.82], body_part="/body")
        
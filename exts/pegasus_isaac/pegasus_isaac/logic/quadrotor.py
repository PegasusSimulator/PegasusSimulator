#!/usr/bin/env python

from pegasus_isaac.logic.vehicle import Vehicle
from pegasus_isaac.logic.sensors import Barometer, IMU, GPS, Magnetometer

class Quadrotor(Vehicle):

    def __init__(self, stage_prefix: str, usd_file: str):
        
        # Initiate the Vehicle
        super.__init__(stage_prefix, usd_file)

        # Create the sensors that a quadrotor typically has
        self._barometer = Barometer()
        self._imu = IMU()
        self._magnetometer = Magnetometer()
        self._gps = GPS()


    def apply_forces(self):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type
        """
        pass
#!/usr/bin/env python
import numpy as np
from pymavlink import mavutil

class MavlinkInterface:

    def __init__(self, connection: str):

        # Connect to the mavlink server
        self._connection = mavutil.mavlink_connection(connection)

        # GPS constants
        self._GPS_fix_type: int = int(3)
        self._GPS_satellites_visible = int(10)
        self._GPS_eph: int = int(1)
        self._GPS_epv: int = int(1)

    def send_heartbeat(self):
        """
        Method that is used to publish an heartbear through mavlink protocol
        """

        self._connection.mav.hearbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)

    def send_gps(self, time_usec, gps_data):
        """
        Method that is used to send simulated GPS data through the mavlink protocol. Receives as argument
        a dictionary with the simulated gps data
        """
        
        # Latitude, longitude and altitude (all in integers)
        lat = int(np.degrees(gps_data["latitude"])*10000000)
        long = int(np.degrees(gps_data["longitude"])*10000000)
        alt = int(gps_data["altitude"] * 1000)
        
        # Estimated velocity (all in integers)
        vel = int(gps_data["speed"] * 100)
        vn = int(gps_data["velocity_north"] * 100)
        ve = int(gps_data["velocity_east"] * 100)
        vd = int(gps_data["velocity_down"] * 100)

        # cog factor in degrees (and expressed in integers)
        cog = np.degrees(np.arctan2(ve, vn))
        
        if cog < 0.:
            cog = cog + 360

        cog = cog * 100

        self._connection.mav.hil_gps_send(
            time_usec,
            self._GPS_fix_type,
            lat, long, alt,
            self._GPS_eph, 
            self._GPS_epv,
            vel, vn, ve, vd,
            cog, self._GPS_satellites_visible
        )

    def send_sensors(self, time_usec, imu_data, mag_data, baro_data):

        # Get the accelerometer and gyro data from the simulated imu
        # converted from ENU to NED convention
        xacc =  imu_data["angular_velocity"][0]
        yacc = -imu_data["angular_velocity"][1]
        zacc = -imu_data["angular_velocity"][2]

        # Get the magnetic field from the simulated magnetometer

        # Get the pressure and temperature from simulated barometer

        # Setup the update code for the mavlink message 

        self._connection.mav.hil_sensor_send(
            time_usec, 
            )
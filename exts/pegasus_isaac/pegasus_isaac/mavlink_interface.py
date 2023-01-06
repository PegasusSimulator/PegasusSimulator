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

    def send_heartbeat(self, mav_type=mavutil.mavlink.MAV_TYPE_GENERIC):
        """
        Method that is used to publish an heartbear through mavlink protocol
        """

        # Note: to know more about these functions, go to pymavlink->dialects->v20->standard.py
        # This contains the definitions for sending the hearbeat and simulated sensor messages
        self._connection.mav.hearbeat_send(
            mav_type,
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

        # Course over ground (NOT heading, but direction of movement), 
        # 0.0..359.99 degrees. If unknown, set to: 65535 [cdeg] (type:uint16_t)
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

    def send_vision_position(self, time_usec, vision_mocap_data):
        """
        This is usefull if simulating a visual inertial plugin running inside Isaac SIM (maybe in the future), or
        to just simulate sending the data that a MOCAP system would send to the vehicle for fusion
        """

        # TODO - implement this feature and handle the ENU to NED convention required by PX4
        x = 0.0         # Local X global position in inertial frame [m] (type:float)
        y = 0.0         # Local Y global position in inertial frame [m] (type:float)
        z = 0.0         # Local Z global position in inertial frame [m] (type:float)
        roll = 0.0      # Roll angle [rad] (type:float)
        pitch = 0.0     # Pitch angle [rad] (type:float)
        yaw = 0.0       # Yaw angle [rad] (type:float)

        # Row-major representation of pose 6x6 cross-covariance matrix upper right triangle 
        # (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries 
        # are the second ROW, etc.). If unknown, assign NaN value to first element in the array. (type:float)
        covariance=(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        
        # Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions 
        # (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system 
        # detects a loop-closure and the estimate jumps. (type:uint8_t)
        reset_counter = 0

        # Note: to know more about these functions, go to pymavlink->dialects->v20->standard.py
        # This contains the definitions for sending the hearbeat and simulated sensor messages
        self._connection.mav.global_vision_position_estimate_send(
            time_usec,        # Timestamp (UNIX time or time since system boot) [us] (type:uint64_t)
            x, y, z,
            roll, pitch, yaw,
            covariance,   
            reset_counter)

    def send_sim_data(self, time_usec, imu_data, mag_data, baro_data, gps_data):
        
        # TODO

        self._connection.mav.sim_state_send(
            q1, q2, q3, q4,                 # w, x, y, z convention
            roll, pitch, yaw,               # rad
            xacc, yacc, zacc,               # m/s^2
            xgyro, ygyro, zgyro,            # rad/s
            lat, lon, alt,                  # [deg, deg, m]
            std_dev_horz, std_dev_vert,     # Horizontal/Vertical position standard deviation
            vn, ve, vd                      # Velocity north, east, down m/s
        )
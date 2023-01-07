#!/usr/bin/env python
import carb
import time
import numpy as np
from threading import Thread, Lock
from pymavlink import mavutil

class MavlinkInterface:

    def __init__(self, connection: str, enable_lockstep: bool = True):

        # Connect to the mavlink server
        self._connection = mavutil.mavlink_connection(connection)
        self._update_rate: float = 250.0 # Hz
        self._is_running: bool = False

        # GPS constants
        self._GPS_fix_type: int = int(3)
        self._GPS_satellites_visible = int(10)
        self._GPS_eph: int = int(1)
        self._GPS_epv: int = int(1)

        # Select whether lockstep is enabled
        self._enable_lockstep: bool = enable_lockstep

        # Auxiliar variables to handle the lockstep between receiving sensor data and actuator control
        self._received_first_imu: bool = False
        self._received_first_actuator: bool = False

        self._received_imu: bool = False        
        self._received_actuator: bool = False

        # Auxiliar variables to check if we have already received an hearbeat from the software in the loop simulation
        self._received_first_hearbeat: bool = False
        self._first_hearbeat_lock: Lock = Lock()

        self._last_heartbeat_sent_time = 0

        # Create a thread for polling for mavlink messages (but only start after receiving the first hearbeat)
        self._update_thread: Thread = Thread(target=self.mavlink_update)

        # Start a new thread that will wait for a new hearbeat
        Thread(target=self.wait_for_first_hearbeat).start()

    def __del__(self):
        # When this object gets destroyed, close the mavlink connection to free the communication port
        if self._connection is not None:
            self._connection.close()

    def start_stream(self):
        self._is_running = True

    def stop_stream(self):
        self._is_running = False

    def update_imu(self, imu_data):
        self._received_imu = True

    def wait_for_first_hearbeat(self):
        """
        Method that is responsible for waiting for the first hearbeat. This method is locking and will only return
        if an hearbeat is received via mavlink. When this first heartbeat is received, a new thread will be created to
        poll for mavlink messages
        """

        carb.log_warn("Waiting for first hearbeat")
        self._connection.wait_heartbeat()

        # Set the first hearbeat flag to true
        with self._first_hearbeat_lock:
            self._received_first_hearbeat = True
        
        carb.log_warn("Received first hearbeat")

        # Start updating mavlink messages at a fixed rate
        self._update_thread.start()

    def mavlink_update(self):
        """
        Method that is running in a thread in parallel to send the mavlink data 
        """

        # Run this thread forever at a fixed rate
        while True:
            
            # Check if we have already received IMU data. If so, we start the lockstep and wait for more imu data
            if self._received_first_imu:
                while not self._received_imu and self._is_running:
                    # TODO - here
                    pass
            
            self._received_imu = False        

            # Check if we have received any mavlink messages
            self.poll_mavlink_messages()

            # Send hearbeats at 1Hz
            if (time.time() - self._last_heartbeat_sent_time) > 1.0 or self._received_first_hearbeat == False:
                self.send_heartbeat()
                self._last_heartbeat_sent_time = time.time()

            # Send sensor messages
            self.send_sensor_msgs()            

            # Send groundtruth
            self.send_ground_truth()

            # TODO - handle mavlink disconnections from the SITL here (TO BE DONE)

            # Handle the control input to the motors
            self.handle_control()
            
            # Update at 250Hz
            time.sleep(1.0/self._update_rate)
        

    def poll_mavlink_messages(self):
        """
        Method that is used to check if new mavlink messages were received
        """

        # If we have not received the first hearbeat yet, do not poll for mavlink messages
        with self._first_hearbeat_lock:
            if self._received_first_hearbeat == False:
                return

        # Check if we need to lock and wait for actuator control data
        needs_to_wait_for_actuator: bool = self._received_first_actuator and self._enable_lockstep

        # Start by assuming that we have not received data for the actuators for the current step
        self._received_actuator = False

        # Use this loop to emulate a do-while loop (make sure this runs at least once)
        while True:
            
            # Try to get a message
            msg = self._connection.recv_match(blocking=needs_to_wait_for_actuator)

            if msg is not None:
                carb.log_warn(msg.controls)

                # TODO - check if message is of the type actuator, and if so, then:
                # self._received_first_actuator = True
                # self._received_actuator = True

            # Check if we do not need to wait for an actuator message or we just received actuator input
            # If so, break out of the infinite loop
            if not needs_to_wait_for_actuator or self._received_actuator:
                break

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

    def send_sensor_msgs(self):
        # TODO
        pass

    def send_ground_truth(self):
        # TODO
        pass

    def handle_control(self):
        #TODO
        pass

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
        
        if cog < 0:
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
        """
        The IMU, MAG and BAROMETER readings in SI units in NED body frame
        time_usec                 : Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
        --------
        IMU_DATA
        --------
        xacc                      : X acceleration [m/s/s] (type:float)
        yacc                      : Y acceleration [m/s/s] (type:float)
        zacc                      : Z acceleration [m/s/s] (type:float)
        xgyro                     : Angular speed around X axis in body frame [rad/s] (type:float)
        ygyro                     : Angular speed around Y axis in body frame [rad/s] (type:float)
        zgyro                     : Angular speed around Z axis in body frame [rad/s] (type:float)
        --------
        MAG_DATA
        --------
        xmag                      : X Magnetic field [gauss] (type:float)
        ymag                      : Y Magnetic field [gauss] (type:float)
        zmag                      : Z Magnetic field [gauss] (type:float)
        ---------
        BARO_DATA
        ---------
        abs_pressure              : Absolute pressure [hPa] (type:float)
        diff_pressure             : Differential pressure (airspeed) [hPa] (type:float)
        pressure_alt              : Altitude calculated from pressure (type:float)
        temperature               : Temperature [degC] (type:float)
        ---------
        fields_updated            : Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim. (type:uint32_t)
        id                        : Sensor ID (zero indexed). Used for multiple sensor inputs (type:uint8_t)
        """

        # Get the accelerometer and gyro data from the simulated imu
        # converted from ENU to NED convention
        xgyro =  imu_data["angular_velocity"][0]
        ygyro = -imu_data["angular_velocity"][1]
        zgyro = -imu_data["angular_velocity"][2]

        # Get the magnetic field from the simulated magnetometer


        # Get the pressure and temperature from simulated barometer

        # Setup the update code for the mavlink message 

        #self._connection.mav.hil_sensor_send(
        #    time_usec, 
        #    )

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
        """
        Status of simulation environment

        q1, q2, q3, q4             : True attitude quaternion components, w, x, y, z (type:float)
        roll, pitch, yaw           : Attitude roll, pitch, yaw expressed as Euler angles
        xacc, yacc, zacc           : X, Y, Z acceleration [m/s/s] (type:float)
        xgyro, ygyro, zgyro        : Angular speed around X, Y, Z axis [rad/s] (type:float)
        lat, lon, alt              : Latitude [deg], Longitude [deg], Altitude [m] (type:float)
        std_dev_horz, std_dev_vert : Horizontal/Vertical position standard deviation (type:float)
        vn, ve, vd                 : True velocity in north, east, down direction in earth-fixed NED frame [m/s] (type:float)
        ve                        : True velocity in east direction in earth-fixed NED frame [m/s] (type:float)
        vd                        : True velocity in down direction in earth-fixed NED frame [m/s] (type:float)

        """
        # TODO
        
        # Quaternion with the orientation of the vehicle
        q1 = 1.0
        q2 = 0.0
        q3 = 0.0
        q4 = 0.0

        # Euler angles with the orientation of the vehicle
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        # Linear acceleration measured by the im
        xacc = 0.0
        yacc = 0.0
        zacc = 0.0

        # Angular velocity measured by the imu
        # converted from ENU to NED convention
        xgyro =  imu_data["angular_velocity"][0]
        ygyro = -imu_data["angular_velocity"][1]
        zgyro = -imu_data["angular_velocity"][2]

        # Get the latitude and longitude from the GPS data 
        lat = np.degrees(gps_data["latitude"])
        lon = np.degrees(gps_data["longitude"])
        alt = gps_data["altitude"]

        # Set the standard deviation to a fixed value (zero for now - fix later)
        std_dev_horz = 0
        std_dev_vert = 0

        # Get the velocity measured by the GPS
        vn = gps_data["velocity_north"]
        ve = gps_data["velocity_east"]
        vd = gps_data["velocity_down"]

        self._connection.mav.sim_state_send(
            q1, q2, q3, q4,                 # w, x, y, z convention
            roll, pitch, yaw,               # rad
            xacc, yacc, zacc,               # m/s^2
            xgyro, ygyro, zgyro,            # rad/s
            lat, lon, alt,                  # [deg, deg, m]
            std_dev_horz, std_dev_vert,     # Horizontal/Vertical position standard deviation
            vn, ve, vd                      # Velocity north, east, down m/s
        )
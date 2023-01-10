#!/usr/bin/env python
import carb
import time
import numpy as np
from threading import Thread, Lock
from pymavlink import mavutil

class SensorSource:
    """
    The binary codes to signal which simulated data is being sent through mavlink
    """
    ACCEL: int = 7          # 0b0000000000111
    GYRO: int = 56          # 0b0000000111000
    MAG: int = 448          # 0b0000111000000
    BARO: int = 6656        # 0b1101000000000
    DIFF_PRESS: int = 1024  # 0b0010000000000

class SensorMsg:
    """
    Class that defines the sensor data that can be sent through mavlink
    """

    def __init__(self):

        # IMU Data
        self.new_imu_data: bool = False
        self.received_first_imu: bool = False
        self.xacc: float = 0.0
        self.yacc: float = 0.0
        self.zacc: float = 0.0 
        self.xgyro: float = 0.0 
        self.ygyro: float = 0.0 
        self.zgyro: float = 0.0

        # Baro Data
        self.new_bar_data: bool = False
        self.abs_pressure: float = 0.0
        self.pressure_alt: float = 0.0
        self.temperature: float = 0.0

        # Magnetometer Data
        self.new_mag_data: bool = False
        self.xmag: float = 0.0
        self.ymag: float = 0.0
        self.zmag: float = 0.0

        # Airspeed Data
        self.new_press_data: bool = False
        self.diff_pressure: float = 0.0

        # GPS Data
        self.new_gps_data: bool = False
        self.fix_type: int = 0
        self.latitude_deg: float = -999
        self.longitude_deg: float = -999
        self.altitude: float = -999
        self.eph: float = 1.0
        self.epv: float = 1.0
        self.velocity: float = 0.0
        self.velocity_north: float = 0.0
        self.velocity_east: float = 0.0
        self.velocity_down: float = 0.0
        self.cog: float = 0.0
        self.satellites_visible: int = 0

        # Vision Pose
        self.new_vision_data: bool = False
        self.vision_x: float = 0.0
        self.vision_y: float = 0.0
        self.vision_z: float = 0.0
        self.vision_roll: float = 0.0
        self.vision_pitch: float = 0.0
        self.vision_yaw: float = 0.0
        self.vision_covariance = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


class ThrusterControl:
    """
    Class that defines saves the thrusters command data received via mavlink
    """

    def __init__(
        self, num_rotors: int=4, 
        input_offset=[0, 0, 0, 0], 
        input_scaling=[0, 0, 0, 0], 
        zero_position_armed=[100, 100, 100, 100],
        min_rotor_velocity=[0, 0, 0, 0],
        max_rotor_velocity=[1100, 1100, 1100, 1100],
        rotor_constant=[5.84E-6, 5.84E-6, 5.84E-6, 5.84E-6]):
        
        self.num_rotors: int = num_rotors
        
        # Values to scale and offset the rotor control inputs received from PX4
        assert len(input_offset) == self.num_rotors
        self.input_offset = input_offset

        assert len(input_scaling) == self.num_rotors
        self.input_scaling = input_scaling

        assert len(zero_position_armed) == self.num_rotors
        self.zero_position_armed = zero_position_armed

        # Values for the minimum and maximum rotor velocity in rad/s
        assert len(min_rotor_velocity) == self.num_rotors
        self.min_rotor_velocity = min_rotor_velocity

        assert len(max_rotor_velocity) == self.num_rotors
        self.max_rotor_velocity = max_rotor_velocity

        # The constant used to translate between rotor velocity (rad/s) and force (N)
        assert len(rotor_constant) == self.num_rotors
        self.rotor_constant = rotor_constant

        # The actual speed references to apply to the vehicle rotor joints
        self._input_reference = [0.0 for i in range(self.num_rotors)]

        # The actual force to apply to each vehicle rotor joints
        self._input_force_reference = [0.0 for i in range(self.num_rotors)]


    @property
    def input_reference(self):
        return self._input_reference

    @property
    def input_force_reference(self):
        return self._input_force_reference
    
    def update_input_reference(self, controls):
        
        # Check if the number of controls received is correct
        if len(controls) < self.num_rotors:
            carb.log_warn("Did not receive enough inputs for all the rotors")
            return

        # Update the desired reference for every rotor (and saturate according to the min and max values)
        for i in range(self.num_rotors):

            # Compute the actual velocity reference to apply to each rotor
            self._input_reference[i] = np.maximum(self.min_rotor_velocity[i], np.minimum((controls[i] + self.input_offset[i]) * self.input_scaling[i] + self.zero_position_armed[i], self.max_rotor_velocity[i]))
        
            # Compute the force that each rotor would generate for that particular rotation velocity
            self._input_force_reference[i] = np.power(self._input_reference[i], 2) * self.rotor_constant[i]

            carb.log_warn(self._input_force_reference[i])

    def zero_input_reference(self):
        """
        When this method is called, the input_reference is updated such that every rotor is stopped
        """
        self._input_reference = [0.0 for i in range(self.num_rotors)]
        self._input_force_reference = [0.0 for i in range(self.num_rotors)]

class MavlinkInterface:

    def __init__(self, connection: str, num_thrusters:int = 4, enable_lockstep: bool = True):

        # Connect to the mavlink server
        self._connection_port = connection
        self._connection = mavutil.mavlink_connection(self._connection_port)
        
        self._update_rate: float = 250.0                    # Hz
        self._time_step: float = 1.0 / self._update_rate    # s
        
        self._is_running: bool = False

        # GPS constants
        self._GPS_fix_type: int = int(3)
        self._GPS_satellites_visible = int(10)
        self._GPS_eph: int = int(1)
        self._GPS_epv: int = int(1)

        # Vehicle Sensor data to send through mavlink
        self._sensor_data: SensorMsg = SensorMsg()

        # Vehicle Rotor data received from mavlink
        self._rotor_data: ThrusterControl = ThrusterControl(
            num_rotors=num_thrusters, 
            input_offset=[0.0, 0.0, 0.0, 0.0],
            input_scaling=[1000.0, 1000.0, 1000.0, 1000.0],
            zero_position_armed=[100.0, 100.0, 100.0, 100.0])

        # Vehicle actuator control data
        self._num_inputs: int = num_thrusters
        self._input_reference: np.ndarray = np.zeros((self._num_inputs,))
        self._armed: bool = False

        self._input_offset: np.ndarray = np.zeros((self._num_inputs,))
        self._input_scaling: np.ndarray = np.zeros((self._num_inputs,))

        # Select whether lockstep is enabled
        self._enable_lockstep: bool = enable_lockstep

        # Auxiliar variables to handle the lockstep between receiving sensor data and actuator control
        self._received_first_actuator: bool = False

        self._received_actuator: bool = False

        # Auxiliar variables to check if we have already received an hearbeat from the software in the loop simulation
        self._received_first_hearbeat: bool = False
        self._first_hearbeat_lock: Lock = Lock()

        self._last_heartbeat_sent_time = 0

        # Auxiliar variables for setting the u_time when sending sensor data to px4
        self._current_utime: int = 0

    def update_imu_data(self, data):

        # Acelerometer data
        self._sensor_data.xacc = data["linear_acceleration"][0]
        self._sensor_data.yacc = data["linear_acceleration"][1]
        self._sensor_data.zacc = data["linear_acceleration"][2]

        # Gyro data
        self._sensor_data.xgyro = data["angular_velocity"][0]
        self._sensor_data.ygyro = data["angular_velocity"][1]
        self._sensor_data.zgyro = data["angular_velocity"][2]

        # Signal that we have new IMU data
        self._sensor_data.new_imu_data = True
        self._sensor_data.received_first_imu = True

    def update_gps_data(self, data):
    
        # GPS data
        self._sensor_data.fix_type = int(data["fix_type"])
        self._sensor_data.latitude_deg = int(data["latitude"] * 10000000)
        self._sensor_data.longitude_deg = int(data["longitude"] * 10000000)
        self._sensor_data.altitude = int(data["altitude"] * 1000)
        self._sensor_data.eph = int(data["eph"])
        self._sensor_data.epv = int(data["epv"])
        self._sensor_data.velocity = int(data["speed"] * 100)
        self._sensor_data.velocity_north = int(data["velocity_north"] * 100)
        self._sensor_data.velocity_east = int(data["velocity_east"] * 100)
        self._sensor_data.velocity_down = int(data["velocity_down"] * 100)
        self._sensor_data.cog = int(data["cog"] * 100)
        self._sensor_data.satellites_visible = int(data["sattelites_visible"])

        # Signal that we have new GPS data
        self._sensor_data.new_gps_data = True

    def update_bar_data(self, data):
        
        # Barometer data
        self._sensor_data.temperature = data["temperature"]
        self._sensor_data.abs_pressure = data["absolute_pressure"]
        self._sensor_data.pressure_alt = data["pressure_altitude"]

        # Signal that we have new Barometer data
        self._sensor_data.new_bar_data = True

    def update_mag_data(self, data):

        # Magnetometer data
        self._sensor_data.xmag = data["magnetic_field"][0]
        self._sensor_data.ymag = data["magnetic_field"][1]
        self._sensor_data.zmag = data["magnetic_field"][2]  

        # Signal that we have new Magnetometer data
        self._sensor_data.new_mag_data = True

    def update_vision_data(self, data):

        # Vision or MOCAP data
        self._sensor_data.vision_x = data["x"]
        self._sensor_data.vision_y = data["y"]
        self._sensor_data.vision_z = data["z"]
        self._sensor_data.vision_roll = data["roll"]
        self._sensor_data.vision_pitch = data["pitch"]
        self._sensor_data.vision_yaw = data["yaw"]
        
        # Signal that we have new vision or mocap data
        self._sensor_data.new_vision_data = True

    def __del__(self):

        # When this object gets destroyed, close the mavlink connection to free the communication port
        try:
            self._connection.close()
        except:
            carb.log_info("Mavlink connection was not closed, because it was never opened")

    def start_stream(self):
        
        # If we are already running the mavlink interface, then ignore the function call
        if self._is_running == True:
            return

        # If the connection no longer exists (we stoped and re-started the stream, then re_intialize the interface)
        if self._connection is None:
            self.re_initialize_interface()
        
        # Set the flag to signal that the mavlink transmission has started
        self._is_running = True

        # Create a thread for polling for mavlink messages (but only start after receiving the first hearbeat)
        self._update_thread: Thread = Thread(target=self.mavlink_update)

        # Start a new thread that will wait for a new hearbeat
        Thread(target=self.wait_for_first_hearbeat).start()

    def stop_stream(self):
        
        # If the simulation was already stoped, then ignore the function call
        if self._is_running == False:
            return

        # Set the flag so that we are no longer running the mavlink interface
        self._is_running = False

        # Terminate the infinite thread loop
        self._update_thread.join()

        # Close the mavlink connection
        self._connection.close()
        self._connection = None

    def re_initialize_interface(self):

        self._is_running = False

        # Restart the sensor daata
        self._sensor_data = SensorMsg()
        
        # Restart the connection
        self._connection = mavutil.mavlink_connection(self._connection_port)

        # Auxiliar variables to handle the lockstep between receiving sensor data and actuator control
        self._received_first_actuator: bool = False
        self._received_actuator: bool = False

        # Auxiliar variables to check if we have already received an hearbeat from the software in the loop simulation
        self._received_first_hearbeat: bool = False
        self._first_hearbeat_lock: Lock = Lock()

        self._last_heartbeat_sent_time = 0

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
        while self._is_running:
            
            # Check if we have already received IMU data. If not, start the lockstep and wait for more data
            if self._sensor_data.received_first_imu:
                while not self._sensor_data.new_imu_data and self._is_running:
                    # Sleep and then try to check if we have new simulated sensor data 
                    time.sleep(1.0 / self._update_rate)

            # Check if we have received any mavlink messages
            self.poll_mavlink_messages()

            # Send hearbeats at 1Hz
            if (time.time() - self._last_heartbeat_sent_time) > 1.0 or self._received_first_hearbeat == False:
                self.send_heartbeat()
                self._last_heartbeat_sent_time = time.time()

            # Update the current u_time for px4
            self._current_utime += int(self._time_step * 1000000)

            # Send sensor messages
            self.send_sensor_msgs(self._current_utime)

            # Send the GPS messages
            self.send_gps_msgs(self._current_utime)        

            # Send groundtruth
            self.send_ground_truth()

            # TODO - handle mavlink disconnections from the SITL here (TO BE DONE)

            # Handle the control input to the motors
            #self.handle_control()
            
            # Update at 250Hz or more
            time.sleep(1.0 / self._update_rate)
        

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

            # If a message was received
            if msg is not None:

                # Check if it is of the type that contains actuator controls
                if msg.id == mavutil.mavlink.MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                    
                    self._received_first_actuator = True
                    self._received_actuator = True

                    # Handle the control of the actuation commands received by PX4
                    self.handle_control(msg.time_usec, msg.controls, msg.mode, msg.flags)

            # Check if we do not need to wait for an actuator message or we just received actuator input
            # If so, break out of the infinite loop
            if not needs_to_wait_for_actuator or self._received_actuator:
                break

    def send_heartbeat(self, mav_type=mavutil.mavlink.MAV_TYPE_GENERIC):
        """
        Method that is used to publish an heartbear through mavlink protocol
        """

        carb.log_warn("Sending heartbeat")

        # Note: to know more about these functions, go to pymavlink->dialects->v20->standard.py
        # This contains the definitions for sending the hearbeat and simulated sensor messages
        self._connection.mav.heartbeat_send(
            mav_type,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)

    def send_sensor_msgs(self, time_usec: int):
        """
        Method that when invoked, will send the simulated sensor data through mavlink
        """
        carb.log_warn("Sending sensor msgs")

        # Check which sensors have new data to send
        fields_updated: int = 0

        if self._sensor_data.new_imu_data:
            # Set the bit field to signal that we are sending updated accelerometer and gyro data
            fields_updated = fields_updated | SensorSource.ACCEL | SensorSource.GYRO
            self._sensor_data.new_imu_data = False

        if self._sensor_data.new_mag_data:
            # Set the bit field to signal that we are sending updated magnetometer data
            fields_updated = fields_updated | SensorSource.MAG
            self._sensor_data.new_mag_data = False

        if self._sensor_data.new_bar_data:
            # Set the bit field to signal that we are sending updated barometer data
            fields_updated = fields_updated | SensorSource.BARO
            self._sensor_data.new_bar_data = False

        if self._sensor_data.new_press_data:
            # Set the bit field to signal that we are sending updated diff pressure data
            fields_updated = fields_updated | SensorSource.DIFF_PRESS
            self._sensor_data.new_press_data = False

        try:
            self._connection.mav.hil_sensor_send(
                time_usec,
                self._sensor_data.xacc,
                self._sensor_data.yacc,
                self._sensor_data.zacc,
                self._sensor_data.xgyro,
                self._sensor_data.ygyro,
                self._sensor_data.zgyro,
                self._sensor_data.xmag,
                self._sensor_data.ymag,
                self._sensor_data.zmag,
                self._sensor_data.abs_pressure,
                self._sensor_data.diff_pressure,
                self._sensor_data.pressure_alt,
                self._sensor_data.altitude,
                fields_updated
            )
        except:
            carb.log_warn("Could not send sensor data through mavlink")

    def send_gps_msgs(self, time_usec: int):
        """
        Method that is used to send simulated GPS data through the mavlink protocol.
        """
        carb.log_warn("Sending GPS msgs")

        # Do not send GPS data, if no new data was received
        if not self._sensor_data.new_gps_data:
            return
        
        self._sensor_data.new_gps_data = False
        
        # Latitude, longitude and altitude (all in integers)
        try:
            self._connection.mav.hil_gps_send(
                time_usec,
                self._sensor_data.fix_type,
                self._sensor_data.latitude_deg,
                self._sensor_data.longitude_deg,
                self._sensor_data.altitude,
                self._sensor_data.eph,
                self._sensor_data.epv,
                self._sensor_data.velocity,
                self._sensor_data.velocity_north,
                self._sensor_data.velocity_east,
                self._sensor_data.velocity_down,
                self._sensor_data.cog,
                self._sensor_data.satellites_visible
            )
        except:
            carb.log_warn("Could not send gps data through mavlink")

    def send_vision_msgs(self, time_usec: int):
        """
        Method that is used to send simulated vision/mocap data through the mavlink protocol.
        """
        carb.log_warn("Sending vision/mocap msgs")

        # Do not send vision/mocap data, if not new data was received
        if not self._sensor_data.new_vision_data:
            return

        self._sensor_data.new_vision_data = False

        try:
            self._connection.mav.global_vision_position_estimate_send(
                time_usec,
                self._sensor_data.vision_x,
                self._sensor_data.vision_y,
                self._sensor_data.vision_z,
                self._sensor_data.vision_roll,
                self._sensor_data.vision_pitch,
                self._sensor_data.vision_yaw,
                self._sensor_data.vision_covariance)
        except:
            carb.log_warn("Could not send vision/mocap data through mavlink")

    def send_ground_truth(self):
        # TODO
        pass

    def handle_control(self, time_usec, controls, mode, flags):
        """
        Method that when received a control message, compute the forces simulated force that should be applied
        on each rotor of the vehicle
        """
        
        # Check if the vehicle is armed - Note: here we have to add a +1 since the code for armed is 128, but
        # pymavlink is return 129 (the end of the buffer)
        if mode == mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED + 1:

            carb.log_warn("Parsing control input")
            
            # Set the rotor target speeds
            self._rotor_data.update_input_reference(controls)
        
        # If the vehicle is not armed, do not rotate the propellers
        else:
            self._rotor_data.zero_input_reference()
            
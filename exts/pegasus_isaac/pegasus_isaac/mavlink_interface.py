#!/usr/bin/env python
import carb
import time
import numpy as np
from threading import Thread, Lock
from pymavlink import mavutil

class MavlinkInterface:

    def __init__(self, connection: str, num_thrusters:int = 4, enable_lockstep: bool = True):

        # Connect to the mavlink server
        self._connection_port = connection
        self._connection = mavutil.mavlink_connection(self._connection_port)
        self._update_rate: float = 250.0 # Hz
        self._is_running: bool = False

        # GPS constants
        self._GPS_fix_type: int = int(3)
        self._GPS_satellites_visible = int(10)
        self._GPS_eph: int = int(1)
        self._GPS_epv: int = int(1)

        # Vehicle Sensor data to send through mavlink
        self._new_imu_data: bool = False
        self._new_gps_data: bool = False
        self._new_bar_data: bool = False
        self._new_mag_data: bool = False

        self._imu_data = None
        self._gps_data = None
        self._bar_data = None
        self._mag_data = None

        # Vehicle actuator control data
        self._num_inputs: int = num_thrusters
        self._input_reference: np.ndarray = np.zeros((self._num_inputs,))
        self._armed: bool = False

        self._input_offset: np.ndarray = np.zeros((self._num_inputs,))
        self._input_scaling: np.ndarray = np.zeros((self._num_inputs,))

        # TODO - input_reference = (mavlink_input + input_offset) * input_scalling + zero_position_armed

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

    """
    Properties
    """
    @property
    def imu_data(self, data):
        self._imu_data = data
        self._new_imu_data = True

    @property
    def gps_data(self, data):
        self._gps_data = data
        self._new_gps_data = True

    @property
    def bar_data(self, data):
        self._bar_data = data
        self._new_bar_data = True

    @property
    def mag_data(self, data):
        self._mag_data = data
        self._new_mag_data = True

    def __del__(self):

        # When this object gets destroyed, close the mavlink connection to free the communication port
        if self._connection is not None:
            self._connection.close()

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

        self._is_running: bool = False
        
        # Restart the connection
        self._connection = mavutil.mavlink_connection(self._connection_port)

        # Auxiliar variables to handle the lockstep between receiving sensor data and actuator control
        self._received_first_imu: bool = False
        self._received_first_actuator: bool = False

        self._received_imu: bool = False        
        self._received_actuator: bool = False

        # Auxiliar variables to check if we have already received an hearbeat from the software in the loop simulation
        self._received_first_hearbeat: bool = False
        self._first_hearbeat_lock: Lock = Lock()

        self._last_heartbeat_sent_time = 0

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
        while self._is_running:
            
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
            carb.log_warn(msg)

            #if msg is not None:
            #    pass

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

        carb.log_warn("Sending heartbeat")

        # Note: to know more about these functions, go to pymavlink->dialects->v20->standard.py
        # This contains the definitions for sending the hearbeat and simulated sensor messages
        self._connection.mav.heartbeat_send(
            mav_type,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)

    def send_sensor_msgs(self):
        """
        Method that when invoked, will send the simulated sensor data through mavlink
        """

        carb.log_warn("Sending sensor msgs")

        xacc = 0.0
        yacc = 0.0
        zacc = 0.0 
        xgyro = 0.0 
        ygyro = 0.0 
        zgyro = 0.0
        xmag = 0.0
        ymag = 0.0
        zmag = 0.0
        abs_pressure = 0.0
        pressure_alt = 0.0
        temperature = 0.0
        diff_pressure = 0.
        fields_updated = 1 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7 | 1<<8 | 1<<9 | 0<<10 | 1<<11 | 1<<12

        try:
            self._connection.mav.hil_sensor_send(int(time.time()),
                                                xacc, yacc, zacc,
                                                xgyro, ygyro, zgyro,
                                                xmag, ymag, zmag,
                                                abs_pressure, diff_pressure, pressure_alt, temperature,
                                                fields_updated)
        except:
            carb.log_warn("Could not send sensor data through mavlink")

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

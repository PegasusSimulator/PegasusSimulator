"""
| File: ardupilot_mavlink_backend.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: File that implements the Mavlink Backend for communication/control with/of the vehicle simulation
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""
__all__ = ["ArduPilotMavlinkBackend", "ArduPilotMavlinkBackendConfig"]

import carb
import time
import math
import numpy as np
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect

from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.backend import Backend
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.tools.ardupilot_launch_tool import ArduPilotLaunchTool
from pegasus.simulator.logic.backends.tools.ArduPilotPlugin import ArduPilotPlugin


class SensorSource:
    """ The binary codes to signal which simulated data is being sent through mavlink

    Atribute:
        | ACCEL (int): mavlink binary code for the accelerometer (0b0000000000111 = 7)
        | GYRO (int): mavlink binary code for the gyroscope (0b0000000111000 = 56)
        | MAG (int): mavlink binary code for the magnetometer (0b0000111000000=448)
        | BARO (int): mavlink binary code for the barometer (0b1101000000000=6656)
        | DIFF_PRESS (int): mavlink binary code for the pressure sensor (0b0010000000000=1024)
    """

    ACCEL: int = 7    
    GYRO: int = 56          
    MAG: int = 448        
    BARO: int = 6656
    DIFF_PRESS: int = 1024


class SensorMsg:
    """
    An auxiliary data class where we write all the sensor data that is going to be sent through mavlink
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

        # Simulation State
        self.new_sim_state: bool = False
        self.sim_attitude = [1.0, 0.0, 0.0, 0.0]  # [w, x, y, z]
        self.sim_acceleration = [0.0, 0.0, 0.0]  # [x,y,z body acceleration]
        self.sim_angular_vel = [0.0, 0.0, 0.0]  # [roll-rate, pitch-rate, yaw-rate] rad/s
        self.sim_lat = 0.0  # [deg]
        self.sim_lon = 0.0  # [deg]
        self.sim_alt = 0.0  # [m]
        self.sim_ind_airspeed = 0.0  # Indicated air speed
        self.sim_true_airspeed = 0.0  # Indicated air speed
        self.sim_velocity_inertial = [0.0, 0.0, 0.0]  # North-east-down [m/s]


class ThrusterControl:  
    """
    An auxiliary data class that saves the thrusters command data received via mavlink and 
    scales them into individual angular velocities expressed in rad/s to apply to each rotor
    """

    def __init__(
        self,
        num_rotors: int = 4,
        input_offset=[0, 0, 0, 0],
        input_scaling=[0, 0, 0, 0],
        zero_position_armed=[100, 100, 100, 100],
    ):
        """Initialize the ThrusterControl object

        Args:
            num_rotors (int): The number of rotors that the actual system has 4.
            input_offset (list): A list with the offsets to apply to the rotor values received via mavlink. Defaults to [0, 0, 0, 0].
            input_scaling (list): A list with the scaling to apply to the rotor values received via mavlink. Defaults to [0, 0, 0, 0].
            zero_position_armed (list): Another list of offsets to apply to the rotor values received via mavlink. Defaults to [100, 100, 100, 100].
        """

        self.num_rotors: int = num_rotors

        # Values to scale and offset the rotor control inputs received from ArduPilot
        assert len(input_offset) == self.num_rotors
        self.input_offset = input_offset

        assert len(input_scaling) == self.num_rotors
        self.input_scaling = input_scaling

        assert len(zero_position_armed) == self.num_rotors
        self.zero_position_armed = zero_position_armed

        # The actual speed references to apply to the vehicle rotor joints
        self._input_reference = [0.0 for i in range(self.num_rotors)]

    @staticmethod
    def rpm_to_rad_per_sec(rpm):
        """
        Convert RPM (Revolutions Per Minute) to radians per second (rad/s).

        Parameters:
        rpm (float): The speed in RPM.

        Returns:
        float: The speed in radians per second (rad/s).
        """
        rad_per_sec = (rpm * 2 * math.pi) / 60
        return rad_per_sec
    

    @staticmethod
    def rad_per_sec_to_rpm(rad_per_sec):
        """
        Converts angular velocity from radians per second to revolutions per minute (RPM).

        :param rad_per_sec: Angular velocity in radians per second (rad/s)
        :return: Angular velocity in revolutions per minute (RPM)
        """
        rpm = rad_per_sec * (60 / (2 * math.pi))
        return rpm

    @staticmethod
    def pwm_to_rpm(pwm):
        """
        Converts a PWM value to RPM using a linear approximation.

        :param pwm: The PWM value in microseconds
        :param pwm_min: The minimum PWM value (e.g., 1000 µs)
        :param pwm_max: The maximum PWM value (e.g., 2000 µs)
        :param rpm_max: The maximum RPM corresponding to pwm_max
        :return: The estimated RPM
        """
        # TODO: config
        pwm_min, pwm_max = (1000, 1500) # microseconds
        rpm_max = ThrusterControl.rad_per_sec_to_rpm(1100) # RPM

        if pwm < pwm_min:
            return 0  # Below minimum PWM, consider it as 0 RPM
        elif pwm > pwm_max:
            return rpm_max  # Above maximum PWM, consider it as max RPM
        else:
            rpm = ((pwm - pwm_min) / (pwm_max - pwm_min)) * rpm_max
            return rpm
    
    @property
    def input_reference(self):
        """A list of floats with the angular velocities in rad/s

        Returns:
            list: A list of floats with the angular velocities to apply to each rotor, expressed in rad/s
        """
        return self._input_reference

    def update_input_reference(self, servos):
        """Takes a list with the thrust controls received via mavlink and scales them in order to generated
        the equivalent angular velocities in rad/s

        Args:
            controls (list): A list of ints with thrust controls received via mavlink
        """

        # Check if the number of controls received is correct
        if len(servos) < self.num_rotors:
            carb.log_warn("Did not receive enough inputs for all the rotors")
            return

        print(f"Servos_original[{servos[0]}, {servos[1]}, {servos[2]}, {servos[3]}]")
            
        # Update the desired reference for every rotor (and saturate according to the min and max values)
        for i in range(self.num_rotors):
                
            # Compute the actual veloci
            # ty reference to apply to each rotor
            # self._input_reference[i] = (rpm_to_rad_per_sec(servos[i]) + self.input_offset[i]) * self.input_scaling[i] + self.zero_position_armed[i]
            # self._input_reference[i] = rpm_to_rad_per_sec(servos[i])
            # ccw_sign = -1 if (i == 0 or i == 1) else 1
            
            print(f"servos[{i}] = {servos[i]}")

            servo_rpm = ThrusterControl.pwm_to_rpm(servos[i])
            print(f"servo_rpm[{i}] = {servo_rpm}")

            self._input_reference[i] = ThrusterControl.rpm_to_rad_per_sec(servo_rpm) # TODO offest

            # TODO upper limit
            # self._input_reference[i] = 800 if self._input_reference[i] > 800 else self._input_reference[i]
            print(f"self._input_reference[{i}] = {self._input_reference[i]})")

            # print(f"self._input_reference[i] = (rpm_to_rad_per_sec(servos[i] + self.input_offset[i]) * self.input_scaling[i] + self.zero_position_armed[i]")
            # print("servos[i] = rpm_to_rad_per_sec(servos[i])")
            # print(f"{servos[i]}:{i} = {rpm_to_rad_per_sec(servos[i])}")
            # print(f"{self._input_reference[i]} = ({rpm_to_rad_per_sec(servos[i])} + {self.input_offset[i]}) * {self.input_scaling[i]} + {self.zero_position_armed[i]}")

        # print(f"Servos[{self._input_reference[0]}, {self._input_reference[1]}, {self._input_reference[2]}, {self._input_reference[3]}]")

    def zero_input_reference(self):
        """
        When this method is called, the input_reference is updated such that every rotor is stopped
        """
        self._input_reference = [0.0 for i in range(self.num_rotors)]


class ArduPilotMavlinkBackendConfig:
    """
    An auxiliary data class used to store all the configurations for the mavlink communications.
    """

    def __init__(self, config={}):
        """
        Initialize the ArduPilotMavlinkBackendConfig class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the Mavlink interface - it can be empty or only have some of the parameters used by this backend.
        
        Examples:
            The dictionary default parameters are

            >>> {"vehicle_id": 0,           
            >>>  "connection_type": "udpin",           
            >>>  "connection_ip": "localhost",
            >>>  "connection_baseport": 5760,
            >>>  "ardupilot_autolaunch": True,
            >>>  "ardupilot_dir": "PegasusInterface().ardupilot_path",
            >>>  "ardupilot_vehicle_model": "gazebo-iris",
            >>>  "enable_lockstep": True,
            >>>  "num_rotors": 4,
            >>>  "input_offset": [0.0, 0.0, 0.0, 0.0],
            >>>  "input_scaling": [1000.0, 1000.0, 1000.0, 1000.0],
            >>>  "zero_position_armed": [100.0, 100.0, 100.0, 100.0],
            >>>  "update_rate": 250.0
            >>> }
        """

        # Configurations for the mavlink communication protocol (note: the vehicle id is sumed to the connection_baseport)
        self.vehicle_id = config.get("vehicle_id", 0)
        self.connection_type = config.get("connection_type", "udpin") # TODO working
        # self.connection_type = config.get("connection_type", "tcp")
        self.connection_ip = config.get("connection_ip", "127.0.0.1") # TODO working
        self.connection_baseport = config.get("connection_baseport", 14550) # TODO working
        # self.connection_baseport = config.get("connection_baseport", 14551)
        # self.connection_baseport = config.get("connection_baseport", 5760)

        # Configure whether to launch ArduPilot in the background automatically or not for every vehicle launched
        self.ardupilot_autolaunch: bool = config.get("ardupilot_autolaunch", True)
        self.ardupilot_dir: str = config.get("ardupilot_dir", PegasusInterface().ardupilot_path)
        self.ardupilot_vehicle_model: str = config.get("ardupilot_vehicle_model", "gazebo-iris")

        # Configurations to interpret the rotors control messages coming from mavlink
        self.enable_lockstep: bool = config.get("enable_lockstep", False)
        self.num_rotors: int = config.get("num_rotors", 4)
        self.input_offset = config.get("input_offset", [-1000.0, -1000.0, -1000.0, -1000.0]) # TODO
        self.input_scaling = config.get("input_scaling", [1.0, 1.0, 1.0, 1.0])
        self.zero_position_armed = config.get("zero_position_armed", [0.0, 0.0, 0.0, 0.0])

        # The update rate at which we will be sending data to mavlink (TODO - remove this from here in the future
        # and infer directly from the function calls)
        self.update_rate: float = config.get("update_rate", 250.0)  # [Hz]


class ArduPilotMavlinkBackend(Backend):
    """ The Mavlink Backend used to receive the vehicle's state and sensor data in order to send to ArduPilot through mavlink. It also
    receives via mavlink the thruster commands to apply to each vehicle rotor.
    """

    def __init__(self, config: ArduPilotMavlinkBackendConfig = ArduPilotMavlinkBackendConfig()):
        """Initialize the ArduPilotMavlinkBackend

        Args:
            config (ArduPilotMavlinkBackendConfig): The configuration class for the ArduPilotMavlinkBackend. Defaults to ArduPilotMavlinkBackendConfig().
        """

        # Initialize the Backend object
        super().__init__(config)

        # Setup the desired mavlink connection port
        # The connection will only be created once the simulation starts
        self._vehicle_id = config.vehicle_id
        self._connection = None
        self._connection_port = f"{config.connection_type}:{config.connection_ip}:{config.connection_baseport}"
        # TODO: config.vehicle_id)

        # Check if we need to autolaunch ArduPilot in the background or not
        self.ardupilot_autolaunch: bool = config.ardupilot_autolaunch
        self.ardupilot_vehicle_model: str = config.ardupilot_vehicle_model  # only needed if ardupilot_autolaunch == True
        self.ardupilot_tool: ArduPilotLaunchTool = None
        self.ardupilot_dir: str = config.ardupilot_dir

        # Set the update rate used for sending the messages (TODO - remove this hardcoded value from here)
        self._update_rate: float = config.update_rate
        self._time_step: float = 1.0 / self._update_rate  # s

        self._is_running: bool = False

        # Vehicle Sensor data to send through mavlink
        self._sensor_data: SensorMsg = SensorMsg()

        # Vehicle Rotor data received from mavlink
        self._rotor_data: ThrusterControl = ThrusterControl(
            config.num_rotors, config.input_offset, config.input_scaling, config.zero_position_armed
        )

        # Vehicle actuator control data
        self._num_inputs: int = config.num_rotors
        self._input_reference: np.ndarray = np.zeros((self._num_inputs,))
        self._armed: bool = False

        self._input_offset: np.ndarray = np.zeros((self._num_inputs,))
        self._input_scaling: np.ndarray = np.zeros((self._num_inputs,))

        # Select whether lockstep is enabled
        self._enable_lockstep: bool = config.enable_lockstep

        # Auxiliar variables to handle the lockstep between receiving sensor data and actuator control
        self._received_first_actuator: bool = False

        self._received_actuator: bool = False

        # Auxiliar variables to check if we have already received an hearbeat from the software in the loop simulation
        self._received_first_hearbeat: bool = False

        self._last_heartbeat_sent_time = 0

        # Auxiliar variables for setting the u_time when sending sensor data to ArduPilot
        self._current_utime: int = 0

        self.packet_num = 0

    def update_sensor(self, sensor_type: str, data):
        """Method that is used as callback for the vehicle for every iteration that a sensor produces new data. 
        Only the IMU, GPS, Barometer and  Magnetometer sensor data are stored to be sent through mavlink. Every other 
        sensor data that gets passed to this function is discarded.

        Args:
            sensor_type (str): A name that describes the type of sensor
            data (dict): A dictionary that contains the data produced by the sensor
        """

        if sensor_type == "IMU":
            self.update_imu_data(data)
        elif sensor_type == "GPS":
            self.update_gps_data(data)
        elif sensor_type == "Barometer":
            self.update_bar_data(data)
        elif sensor_type == "Magnetometer":
            self.update_mag_data(data)
        # If the data received is not from one of the above sensors, then this backend does
        # not support that sensor and it will just ignore it
        else:
            pass

    def update_imu_data(self, data):
        # HIL_SENSOR (107)
        """Gets called by the 'update_sensor' method to update the current IMU data

        Args:
            data (dict): The data produced by an IMU sensor
        """

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
        # HIL_GPS (113)
        """Gets called by the 'update_sensor' method to update the current GPS data

        Args:
            data (dict): The data produced by an GPS sensor
        """

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

        # Also update the groundtruth for the latitude and longitude
        self._sensor_data.sim_lat = int(data["latitude_gt"] * 10000000)
        self._sensor_data.sim_lon = int(data["longitude_gt"] * 10000000)
        self._sensor_data.sim_alt = int(data["altitude_gt"] * 1000)

    def update_bar_data(self, data):
        # # HIL_SENSOR (107)
        """Gets called by the 'update_sensor' method to update the current Barometer data

        Args:
            data (dict): The data produced by an Barometer sensor
        """

        # Barometer data
        self._sensor_data.temperature = data["temperature"]
        self._sensor_data.abs_pressure = data["absolute_pressure"]
        self._sensor_data.pressure_alt = data["pressure_altitude"]

        # Signal that we have new Barometer data
        self._sensor_data.new_bar_data = True

    def update_mag_data(self, data):
        # HIL_SENSOR (107)
        """Gets called by the 'update_sensor' method to update the current Vision data

        Args:
            data (dict): The data produced by an Vision sensor
        """

        # Magnetometer data
        self._sensor_data.xmag = data["magnetic_field"][0]
        self._sensor_data.ymag = data["magnetic_field"][1]
        self._sensor_data.zmag = data["magnetic_field"][2]

        # Signal that we have new Magnetometer data
        self._sensor_data.new_mag_data = True

    def update_vision_data(self, data):
        """Method that 'in the future' will get called by the 'update_sensor' method to update the current Vision data
        This callback is currently not being called (TODO in a future simulator version)
        Args:
            data (dict): The data produced by an Vision sensor
        """

        # Vision or MOCAP data
        self._sensor_data.vision_x = data["x"]
        self._sensor_data.vision_y = data["y"]
        self._sensor_data.vision_z = data["z"]
        self._sensor_data.vision_roll = data["roll"]
        self._sensor_data.vision_pitch = data["pitch"]
        self._sensor_data.vision_yaw = data["yaw"]

        # Signal that we have new vision or mocap data
        self._sensor_data.new_vision_data = True

    def update_state(self, state: State):
        """Method that is used as callback and gets called at every physics step with the current state of the vehicle.
        This state is then stored in order to be sent as groundtruth via mavlink

        Args:
            state (State): The current state of the vehicle.
        """

        # Get the quaternion in the convention [x, y, z, w]
        attitude = state.get_attitude_ned_frd()

        # Rotate the quaternion to the mavlink standard
        self._sensor_data.sim_attitude[0] = attitude[3]
        self._sensor_data.sim_attitude[1] = attitude[0]
        self._sensor_data.sim_attitude[2] = attitude[1]
        self._sensor_data.sim_attitude[3] = attitude[2]

        # Get the angular velocity
        ang_vel = state.get_angular_velocity_frd()
        self._sensor_data.sim_angular_vel[0] = ang_vel[0]
        self._sensor_data.sim_angular_vel[1] = ang_vel[1]
        self._sensor_data.sim_angular_vel[2] = ang_vel[2]

        # Get the acceleration
        acc_vel = state.get_linear_acceleration_ned()
        self._sensor_data.sim_acceleration[0] = int(acc_vel[0] * 1000)
        self._sensor_data.sim_acceleration[1] = int(acc_vel[1] * 1000)
        self._sensor_data.sim_acceleration[2] = int(acc_vel[2] * 1000)

        # Get the latitude, longitude and altitude directly from the GPS

        # Get the linear velocity of the vehicle in the inertial frame
        lin_vel = state.get_linear_velocity_ned()
        self._sensor_data.sim_velocity_inertial[0] = int(lin_vel[0] * 100)
        self._sensor_data.sim_velocity_inertial[1] = int(lin_vel[1] * 100)
        self._sensor_data.sim_velocity_inertial[2] = int(lin_vel[2] * 100)

        # Compute the air_speed - assumed indicated airspeed due to flow aligned with pitot (body x)
        body_vel = state.get_linear_body_velocity_ned_frd()
        self._sensor_data.sim_ind_airspeed = int(body_vel[0] * 100)
        self._sensor_data.sim_true_airspeed = int(np.linalg.norm(lin_vel) * 100)  # TODO - add wind here

        self._sensor_data.new_sim_state = True

    def input_reference(self):
        """Method that when implemented, should return a list of desired angular velocities to apply to the vehicle rotors
        """
        return self._rotor_data.input_reference

    def __del__(self):
        """Gets called when the ArduPilotMavlinkBackend object gets destroyed. When this happens, we make sure
        to close any mavlink connection open for this vehicle.
        """

        # When this object gets destroyed, close the mavlink connection to free the communication port
        try:
            self._connection.close()
            self._connection = None
        except:
            carb.log_info("Mavlink connection was not closed, because it was never opened")

    def start(self):
        """Method that handles the begining of the simulation of vehicle. It will try to open the mavlink connection 
        interface and also attemp to launch ArduPilot in a background process if that option as specified in the config class
        """

        # If we are already running the mavlink interface, then ignore the function call
        if self._is_running == True:
            return

        # If the connection no longer exists (we stoped and re-started the stream, then re_intialize the interface)
        if self._connection is None:
            self.re_initialize_interface()

        # Set the flag to signal that the mavlink transmission has started
        self._is_running = True

        # Launch the ArduPilot in the background if needed
        if self.ardupilot_autolaunch and self.ardupilot_tool is None:
            carb.log_info("Attempting to launch ArduPilot in background process")
            self.ardupilot_tool = ArduPilotLaunchTool(self.ardupilot_dir, self._vehicle_id, self.ardupilot_vehicle_model)
            self.ardupilot_tool.launch_ardupilot()

    def stop(self):
        """Method that when called will handle the stopping of the simulation of vehicle. It will make sure that any open
        mavlink connection will be closed and also that the ArduPilot background process gets killed (if it was auto-initialized)
        """

        # If the simulation was already stoped, then ignore the function call
        if self._is_running == False:
            return

        # Set the flag so that we are no longer running the mavlink interface
        self._is_running = False

        # Close the mavlink connection
        self._connection.close()
        self._connection = None

        # Close the ArduPilot if it was running
        if self.ardupilot_autolaunch and self.ardupilot_autolaunch is not None:
            carb.log_info("Attempting to kill ArduPilot background process")
            self.ardupilot_tool.kill_ardupilot()
            self.ardupilot_tool = None

    def reset(self):
        """For now does nothing. Here for compatibility purposes only
        """
        return

    def re_initialize_interface(self):
        """Auxiliar method used to get the MavlinkInterface to reset the MavlinkInterface to its initial state
        """

        self._is_running = False

        # Restart the sensor data
        self._sensor_data = SensorMsg()

        # Restart the connection
        self._connection = mavutil.mavlink_connection(self._connection_port)

        # Auxiliar variables to handle the lockstep between receiving sensor data and actuator control
        self._received_first_actuator: bool = False
        self._received_actuator: bool = False

        # Auxiliar variables to check if we have already received an hearbeat from the software in the loop simulation
        self._received_first_hearbeat: bool = False

        self._last_heartbeat_sent_time = 0

    def wait_for_first_hearbeat(self):
        """
        Responsible for waiting for the first hearbeat. This method is locking and will only return
        if an hearbeat is received via mavlink. When this first heartbeat is received poll for mavlink messages
        """

        # import pdb;pdb.set_trace()
        # self.send_heartbeat()

        carb.log_warn("Waiting for first hearbeat")
        result = self._connection.wait_heartbeat(blocking=False)

        if result is not None:
            self._received_first_hearbeat = True
            carb.log_warn("Received first hearbeat")

    def update(self, dt):
        """
        Method that is called at every physics step to send data to ArduPilot and receive the control inputs via mavlink

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Check for the first hearbeat on the first few iterations
        if not self._received_first_hearbeat:
            self.wait_for_first_hearbeat()
            return

        # Check if we have already received IMU data. If not, start the lockstep and wait for more data
        if self._sensor_data.received_first_imu:
            while not self._sensor_data.new_imu_data and self._is_running:
                # Just go for the next update and then try to check if we have new simulated sensor data
                # DO not continue and get mavlink thrusters commands until we have simulated IMU data available
                return

        # Wait until the vehicle is armed
        self.check_is_armed()
        
        # Check if we have received any mavlink messages
        self.poll_mavlink_messages()

        # import pdb;pdb.set_trace()
        # Send hearbeats at 1Hz
        if (time.time() - self._last_heartbeat_sent_time) > 1.0 or self._received_first_hearbeat == False:
            # import pdb;pdb.set_trace()
            self.send_heartbeat()
            self._last_heartbeat_sent_time = time.time()

        # Update the current u_time for ArduPilot
        self._current_utime += int(dt * 1000000)

        # Send sensor messages
        self.send_sensor_msgs(self._current_utime)

        # Send the GPS messages
        self.send_gps_msgs(self._current_utime)


    def arming_check_disable(self):
        import pdb;pdb.set_trace()
        self._connection.param_set_send(
            "ARMING_CHECK",
            0, # 0 is disable, 1 is enable  
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        
        message = self._connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)

        if message is not None:
            message = message.to_dict()
            if message["param_id"] == "ARMING_CHECK" and message["param_value"] == 0:
                import pdb;pdb.set_trace()
                carb.log_info("Arming Check is disabled.")
                return True
        
        import pdb;pdb.set_trace()
        carb.log_info("Arming Check is enabled.")
        return False
        
    def check_is_armed(self):
        # Use this loop to emulate a do-while loop (make sure this runs at least once)
        msg = self._connection.recv_match(blocking=False)

        # If a message was received
        if msg is not None:
            if not self._armed:
                if msg.get_type() == 'HEARTBEAT' and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                    self._armed = self._connection.motors_armed()
                    if self._armed:
                        carb.log_warn(">>>>>>>> Drone is armed.")
                    else:
                        carb.log_warn(">>>>>>>> Drone is disarmed.")

    def poll_mavlink_messages(self):
        """
        Method that is used to check if new mavlink messages were received
        """

        # If we have not received the first hearbeat yet, do not poll for mavlink messages
        if self._received_first_hearbeat == False:
            return

        # Check if we need to lock and wait for actuator control data
        # TODO REMOVE: needs_to_wait_for_actuator: bool = self._received_first_actuator and self._enable_lockstep

        # Start by assuming that we have not received data for the actuators for the current step
        self._received_actuator = False
        

        # Use this loop to emulate a do-while loop (make sure this runs at least once)
        if self._armed:
            msg = self._connection.recv_match(blocking=False)

            # Check if it is of the type that contains actuator controls
            # carb.log_warn(">>>>>>>> Waiting for messages...")
            if msg is not None:
                if msg.id == mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                    self._received_first_actuator = True
                    self._received_actuator = True
                    
                    # Handle the control of the actuation commands received by ArduPilot

                    # if self.packet_num % 2 == 0:
                    # servos = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
                    
                    # TODO!!!! Normal (1,2,3,4)
                    # servos = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
                    
                    # Try flipped (3,4,1,2)
                    # servos = [msg.servo3_raw, msg.servo4_raw, msg.servo1_raw, msg.servo2_raw]

                    # Try same (1,1,1,1)
                    servos = [msg.servo1_raw, msg.servo1_raw, msg.servo1_raw, msg.servo1_raw]

                    self.handle_control(servos=servos, is_armed=True)

                    # self.packet_num += 1

                    # Check if we do not need to wait for an actuator message or we just received actuator input
                    # If so, break out of the infinite loop
                    # if not needs_to_wait_for_actuator or self._received_actuator:
                    #     break
        else:
            self.handle_control(servos=[], is_armed=False)

    def send_heartbeat(self, mav_type=mavutil.mavlink.MAV_TYPE_GENERIC):
        """
        Method that is used to publish an heartbear through mavlink protocol

        Args: 
            mav_type (int): The ID that indicates the type of vehicle. Defaults to MAV_TYPE_GENERIC=0 
        """

        carb.log_info("Sending heartbeat")

        # Note: to know more about these functions, go to pymavlink->dialects->v20->standard.py
        # This contains the definitions for sending the hearbeat and simulated sensor messages
        self._connection.mav.heartbeat_send(mav_type, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def send_sensor_msgs(self, time_usec: int):
        """
        Method that when invoked, will send the simulated sensor data through mavlink

        Args:
            time_usec (int): The total time elapsed since the simulation started
        """
        carb.log_info("Sending sensor msgs")

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
                fields_updated,
            )
        except:
            carb.log_warn("Could not send sensor data through mavlink")

    def send_gps_msgs(self, time_usec: int):
        """
        Method that is used to send simulated GPS data through the mavlink protocol.

        Args:
            time_usec (int): The total time elapsed since the simulation started
        """
        carb.log_info("Sending GPS msgs")

        # Do not send GPS data, if no new data was received
        if not self._sensor_data.new_gps_data:
            return

        self._sensor_data.new_gps_data = False

        # Latitude, longitude and altitude (all in integers)
        try:
            response = self._connection.mav.hil_gps_send(
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
                self._sensor_data.satellites_visible,
            )
        except:
            carb.log_warn("Could not send gps data through mavlink")

    def send_vision_msgs(self, time_usec: int):
        """
        Method that is used to send simulated vision/mocap data through the mavlink protocol.

        Args:
            time_usec (int): The total time elapsed since the simulation started
        """
        carb.log_info("Sending vision/mocap msgs")

        # Do not send vision/mocap data, if not new data was received
        if not self._sensor_data.new_vision_data:
            return

        self._sensor_data.new_vision_data = False

        try:
            response = self._connection.mav.global_vision_position_estimate_send(
                time_usec,
                self._sensor_data.vision_x,
                self._sensor_data.vision_y,
                self._sensor_data.vision_z,
                self._sensor_data.vision_roll,
                self._sensor_data.vision_pitch,
                self._sensor_data.vision_yaw,
                self._sensor_data.vision_covariance,
            )
        except:
            carb.log_warn("Could not send vision/mocap data through mavlink")

    def send_ground_truth(self, time_usec: int):
        """
        Method that is used to send the groundtruth data of the vehicle through mavlink

        Args:
            time_usec (int): The total time elapsed since the simulation started
        """

        carb.log_info("Sending groundtruth msgs")

        # Do not send vision/mocap data, if not new data was received
        if not self._sensor_data.new_sim_state or self._sensor_data.sim_alt == 0:
            return

        self._sensor_data.new_sim_state = False

        try:
            response = self._connection.mav.hil_state_quaternion_send(
                time_usec,
                self._sensor_data.sim_attitude,
                self._sensor_data.sim_angular_vel[0],
                self._sensor_data.sim_angular_vel[1],
                self._sensor_data.sim_angular_vel[2],
                self._sensor_data.sim_lat,
                self._sensor_data.sim_lon,
                self._sensor_data.sim_alt,
                self._sensor_data.sim_velocity_inertial[0],
                self._sensor_data.sim_velocity_inertial[1],
                self._sensor_data.sim_velocity_inertial[2],
                self._sensor_data.sim_ind_airspeed,
                self._sensor_data.sim_true_airspeed,
                self._sensor_data.sim_acceleration[0],
                self._sensor_data.sim_acceleration[1],
                self._sensor_data.sim_acceleration[2],
            )
        except:
            carb.log_warn("Could not send groundtruth through mavlink")

    def handle_control(self, servos, is_armed):
        """
        Method that when received a control message, compute the forces simulated force that should be applied
        on each rotor of the vehicle

        Args:
            time_usec (int): The total time elapsed since the simulation started - Ignored argument
            controls (list): A list of ints which contains the thrust_control received via mavlink
            flags: Ignored argument
        """

        # Check if the vehicle is armed - Note: here we have to add a +1 since the code for armed is 128, but
        # pymavlink is return 129 (the end of the buffer) 
        # if mode == mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED + 1:
        if is_armed:
            carb.log_info("Parsing control input")

            # Set the rotor target speeds
            self._rotor_data.update_input_reference(servos)

            # If the vehicle is not armed, do not rotate the propellers
        else:
            # print("Control message has NOT been recieved")
            self._rotor_data.zero_input_reference()

    def update_graphical_sensor(self, sensor_type: str, data):
        """Method that when implemented, should handle the receival of graphical sensor data

        Args:
            sensor_type (str): A name that describes the type of sensor (for example MonocularCamera)
            data (dict): A dictionary that contains the data produced by the sensor
        """
        pass
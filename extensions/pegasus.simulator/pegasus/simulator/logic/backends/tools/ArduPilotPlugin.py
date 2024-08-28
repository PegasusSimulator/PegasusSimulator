import time
import socket
import struct
import json
from dataclasses import dataclass
import numpy as np

class ArduPilotPlugin:
    SERVO_PACKET_SIZE = 40
    SERVO_PACKET_MAGIC = 0x481A
    
    def __init__(self):

        # The address for the flight dynamics model (i.e. this plugin)
        self.fdm_address = '127.0.0.1'
        # The port for the flight dynamics model
        self.fdm_port_in = 9002

        # FCU address and port are auto detected from receving UDP packet from connected client
        # The address for the SITL flight controller
        self.fcu_address = None
        # The port for the SITL flight controller
        self.fcu_port_out = None

        # Last received frame rate from the ArduPilot controller
        self.fcu_frame_rate = 0
        # Last received frame count from the ArduPilot controller
        self.fcu_frame_count = -1

        # Set to false when Gazebo starts to prevent blocking, true when
        # the ArduPilot controller is detected and online, and false if the
        # connection to the ArduPilot controller times out.
        self.arduPilotOnline = False
        
        # Number of consecutive missed ArduPilot controller messages
        self.connectionTimeoutCount = 0
        #  Max number of consecutive missed ArduPilot controller messages before timeout
        self.connectionTimeoutMaxCount = 5  # Example value
        
        # Set true to enforce lock-step simulation
        self.isLockStep = False
       
        # Last sent JSON string, so we can resend if needed.
        self.json_str: str = ""
        
        # Keep track of controller update sim-time.
        self.last_controller_update_time = 0
        # Keep track of the time the last servo packet was received.
        self.last_servo_packet_recv_time = 0
        
        # Sockets
        self.motor_control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.motor_control_sock.setblocking(True)
        self.motor_control_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # self.sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        try:
            self.motor_control_sock.bind((self.fdm_address, self.fdm_port_in))
            print(f"Flight dynamics model @ {self.fdm_address}:{self.fdm_port_in}")

        except socket.error as e:
            print(f"Failed to bind with {self.fdm_address}:{self.fdm_port_in}. Aborting plugin.")

        
    def create_state_json(self, sensor_data, sim_time):
        state = {
            "timestamp": sim_time,
            "imu": {
                "gyro": [
                    sensor_data.xgyro,
                    sensor_data.ygyro,
                    sensor_data.zgyro
                ],
                "accel_body": [
                    sensor_data.xacc,
                    sensor_data.yacc,
                    sensor_data.zacc,
                ]
            },  
            "position": [
               sensor_data.sim_position[0], # X
               sensor_data.sim_position[1], # Y
               sensor_data.sim_position[2], # Z
            ],
            "quaternion": [
               sensor_data.sim_attitude[0], # W
               sensor_data.sim_attitude[1], # X
               sensor_data.sim_attitude[2], # Y
               sensor_data.sim_attitude[3], # Z
            ],
            #   "quaternion": [
            #    1,0,0,0
            # ],
            "velocity": [
              sensor_data.sim_velocity_inertial[0], # X
              sensor_data.sim_velocity_inertial[1], # Y # - sensor_data.sim_velocity_inertial[1], # Y # - sensor_data.sim_velocity_inertial[1], # Y
              sensor_data.sim_velocity_inertial[2]  # Z
            ]
        }

        json_str = json.dumps(state, separators=(',', ':'))
        json_str = "\n" + json_str + "\n"
        json_str = json_str.encode('utf-8')

        self.json_str = json_str
        
        return self.json_str 
        

    def send_state(self):
        # json_str = self.create_state_json()
        # print("Send_State...")
        # print(self.json_str)

        if self.motor_control_sock:
            bytes_sent = self.motor_control_sock.sendto(
                self.json_str,
                (self.fcu_address, self.fcu_port_out)
            )
            # print(f"Sent {bytes_sent} bytes to {self.fcu_address}:{self.fcu_port_out}")

    def unpack_servo_packet(self, data):
        # Ensure the data length is valid
        if len(data) != self.SERVO_PACKET_SIZE:
            raise ValueError(f"Data length must be {self.SERVO_PACKET_SIZE} bytes, got {len(data)} bytes")
        
        # Define the format string according to the structure for little-endian
        # '<HHI16H' specifies:
        #   - < (little-endian byte order)
        #   - HH (2 uint16_t fields)
        #   - I (1 uint32_t field)
        #   - 16H (16 uint16_t fields for pwm values)
        format_string = '<HHI16H'
        
        try:
            # Unpack the data
            unpacked_data = struct.unpack(format_string, data)
            pkt_magic = unpacked_data[0]
            pkt_frame_rate = unpacked_data[1]
            pkt_frame_count = unpacked_data[2]
            pkt_pwm = unpacked_data[3:]
            
            if pkt_magic != self.SERVO_PACKET_MAGIC:
                print(f"Incorrect protocol magic {pkt_magic}, should be {self.SERVO_PACKET_MAGIC}")
                return None, None, None, None
            
            return pkt_magic, pkt_frame_rate, pkt_frame_count, pkt_pwm
        
        except struct.error as e:
            print(f"Unpacking error: {e}")
            return None, None, None, None

    def drain_unread_packets(self):
        """
        Drains all unread packets from the UDP socket.
        """

        # Set socket to non-blocking mode
        self.motor_control_sock.setblocking(False)
        while True:
            try:
                # Attempt to receive data
                data, (client_addr, client_out) = self.motor_control_sock.recvfrom(self.SERVO_PACKET_SIZE)
                if (self.fcu_address is None) or (self.fcu_port_out is None):
                    self.fcu_address = client_addr
                    self.fcu_port_out = client_out

                # print(f"Drained data: {data.decode('utf-8')}")
            except BlockingIOError:
                # No more data to read, exit the loop
                break
            except Exception as e:
                # Handle unexpected exceptions
                print(f"An error occurred while draining packets: {e}")
                break
        
        # Reset the socket to blocking mode
        self.motor_control_sock.setblocking(True)
        print("Drained all packets.")


    def receive_servo_packet(self):
        # Determine wait time based on whether ArduPilot is online
        wait_ms = 10 if self.arduPilotOnline else 1
        wait_sec = wait_ms / 1000.0

        pkt_frame_rate = 0
        pkt_frame_count = 0

        try:
            # Set socket timeout based on wait_ms
            self.motor_control_sock.settimeout(wait_sec)

            # Receive the data and get the client address and port
            data, (client_addr, client_out) = self.motor_control_sock.recvfrom(self.SERVO_PACKET_SIZE)

            # Store the FCU (ArduPilot SITL) address and port if not already set
            if self.fcu_address is None or self.fcu_port_out is None:
                self.fcu_address = client_addr
                self.fcu_port_out = client_out

            # Unpack the received packet
            pkt_magic, pkt_frame_rate, pkt_frame_count, *pkt_pwm = self.unpack_servo_packet(data)
            pkt_pwm = pkt_pwm[0]

            if pkt_magic is None:
                return False, ()

            # print(f"Received packet: magic={pkt_magic}, frame_rate={pkt_frame_rate}, frame_count={pkt_frame_count}, pwm={pkt_pwm}")

        except socket.timeout:
            if self.arduPilotOnline:
                self.connectionTimeoutCount += 1
                if self.connectionTimeoutCount > self.connectionTimeoutMaxCount:
                    self.connectionTimeoutCount = 0
                    if self.isLockStep:
                        print("Send State from receive_servo_packet")
                        self.send_state()
                    else:
                        print("Socket timeout")
                        self.arduPilotOnline = False
                        print(f"Broken ArduPilot connection, resetting motor control.")
                        # self.reset_pids()  # Ensure this method exists and is implemented
            return False, ()

        # Handle ArduPilot online status
        if not self.arduPilotOnline:
            print(f"Connected to ArduPilot controller @ {self.fcu_address}:{self.fcu_port_out}")
            self.arduPilotOnline = True

        # Update frame rate
        self.fcu_frame_rate = pkt_frame_rate

        # Check for controller reset, duplicate frames, or skipped frames
        if pkt_frame_count < self.fcu_frame_count:
            print("ArduPilot controller has reset")
        elif pkt_frame_count == self.fcu_frame_count:
            print("Duplicate input frame")
            if self.isLockStep:
                self.send_state()
            return False, []
        elif pkt_frame_count != self.fcu_frame_count + 1 and self.arduPilotOnline:
            print(f"Missed {pkt_frame_count - self.fcu_frame_count} input frames")

        # Update frame count
        self.fcu_frame_count = pkt_frame_count

        # Reset the connection timeout count
        self.connectionTimeoutCount = 0    

        return True, pkt_pwm
    
    
    # def update_motor_commands(self, pwm):
    #     max_servo_channels = 16

    #     # Compute command based on requested motor speed
    #     for i, control in enumerate(self.controls):
    #         # Enforce limit on the number of control elements
    #         if i < self.MAX_MOTORS:
    #             if control['channel'] < max_servo_channels:
    #                 # Convert PWM to raw command: [servo_min, servo_max] => [0, 1]
    #                 # Default is: [1000, 2000] => [0, 1]
    #                 pwm_value = pwm[control['channel']]
    #                 pwm_min = control['servo_min']
    #                 pwm_max = control['servo_max']
    #                 multiplier = control['multiplier']
    #                 offset = control['offset']

    #                 # Bound incoming command between 0 and 1
    #                 raw_cmd = (pwm_value - pwm_min) / (pwm_max - pwm_min)
    #                 raw_cmd = max(0.0, min(raw_cmd, 1.0))
    #                 control['cmd'] = multiplier * (raw_cmd + offset)

    #                 # Debugging information (uncomment if needed)
    #                 # print(f"apply input chan[{control['channel']}] to control chan[{i}] "
    #                 #       f"with joint name [{control['jointName']}] pwm [{pwm_value}] "
    #                 #       f"raw cmd [{raw_cmd}] adjusted cmd [{control['cmd']}].")
    #             else:
    #                 print(f"[{self.modelName}] control[{i}] channel [{control['channel']}] "
    #                     f"is greater than the number of servo channels [{max_servo_channels}], control not applied.")
    #         else:
    #             print(f"[{self.modelName}] too many motors, skipping [{i} > {self.MAX_MOTORS}].")

    @property
    def sim_time(self):
        return time.time()
    
    def get_sim_duration(self):
        return time.time() - self.sim_time
    
    
    def pre_update(self, sim_time):

        # Update the control surfaces
        recieved, pwms = self.receive_servo_packet()
        if recieved:
            # self.last_servo_packet_recv_time = self.get_sim_duration()
            self.last_servo_packet_recv_time = sim_time

        return recieved, pwms
    
        # if self.dataPtr.arduPilotOnline:
        #     dt = (_info['simTime'] - self.dataPtr.lastControllerUpdateTime).total_seconds()
        #     self.apply_motor_forces(dt, _ecm)

    def post_update(self, sensor_data, sim_time):
        if sim_time > self.last_controller_update_time and self.arduPilotOnline:
            self.last_controller_update_time = sim_time
            self.create_state_json(sim_time=self.last_controller_update_time, sensor_data=sensor_data)
            self.send_state()

  # Mock Sensor Data
    @dataclass
    class SensorData:
        sim_position = [
            -6.874587183616472e-12 + np.random.normal(0, 0.01),
            -1.6699334495870352e-12 + np.random.normal(0, 0.01),
            -0.1949994162458527 + np.random.normal(0, 0.01)
        ]
        sim_attitude = [
            1,
            -4.281847537232883e-12 + np.random.normal(0, 0.001),
            1.7627199589076353e-11 + np.random.normal(0, 0.001),
            -4.281847537232883e-12 + np.random.normal(0, 0.001)
        ]
        sim_velocity_inertial = [
            4.4028248386528135e-12 + np.random.normal(0, 0.01),
            5.46182226507895e-12 + np.random.normal(0, 0.01),
            5.454422342398644e-18 + np.random.normal(0, 0.01)
        ]
        xgyro: float = 4.4028248386528135e-12
        ygyro: float = 5.46182226507895e-12
        zgyro: float = 5.454422342398644e-18
        xacc: float = -1.521417774123255e-9
        yacc: float = 1.976595291657851e-9
        zacc: float = -9.80000000000001

if __name__ == '__main__':
    ap = ArduPilotPlugin()
    ap.drain_unread_packets()

  
    
    while True:
        ap.pre_update()
        time.sleep(0.01)
        ap.post_update(sensor_data=ap.SensorData())
        time.sleep(0.01)

        









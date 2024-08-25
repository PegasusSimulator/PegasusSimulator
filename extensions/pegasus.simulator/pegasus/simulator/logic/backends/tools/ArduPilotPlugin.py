import time
import socket
from pprint import pprint
import struct
import json


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

        self.fdm_frame_rate = 0
        self.fdm_frame_count = 0

        self.arduPilotOnline = False
        self.connectionTimeoutCount = 0
        self.connectionTimeoutMaxCount = 5  # Example value

        self.isLockStep = False
       
        self.json_str: str = ""
        self.state_json = {}

        self.test_sim_time = 0

        self.last_servo_packet_recv_time = 0
        self.last_controller_update_time = 0
        
        # Sockets
        self.motor_control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.motor_control_sock.setblocking(True)
        self.motor_control_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        self.sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        try:
            self.motor_control_sock.bind((self.fdm_address, self.fdm_port_in))
            print(f"Flight dynamics model @ {self.fdm_address}:{self.fdm_port_in}")

            # self.sensor_sock.connect((self.fdm_address, self.fdm_port_in))
        except socket.error as e:
            print(f"Failed to bind with {self.fdm_address}:{self.fdm_port_in}. Aborting plugin.")

        
    def create_state_json(self, sim_time, sensor_data=None): # TODO sensor_data
        # Extract linear acceleration
        # linear_accel = {
        #     'x': imu_msg['linear_acceleration']['x'],
        #     'y': imu_msg['linear_acceleration']['y'],
        #     'z': imu_msg['linear_acceleration']['z']
        # }

        # # Extract angular velocity
        # angular_vel = {
        #     'x': imu_msg['angular_velocity']['x'],
        #     'y': imu_msg['angular_velocity']['y'],
        #     'z': imu_msg['angular_velocity']['z']
        # }

        # # Extract position and orientation
        # position = [
        #     wld_a_to_bdy_a['pos']['x'],
        #     wld_a_to_bdy_a['pos']['y'],
        #     wld_a_to_bdy_a['pos']['z']
        # ]

        # quaternion = [
        #     wld_a_to_bdy_a['rot']['w'],
        #     wld_a_to_bdy_a['rot']['x'],
        #     wld_a_to_bdy_a['rot']['y'],
        #     wld_a_to_bdy_a['rot']['z']
        # ]

        # velocity = [
        #     vel_wld_a['x'],
        #     vel_wld_a['y'],
        #     vel_wld_a['z']
        # ]

        # # Build JSON document
        # state = {
        #     'timestamp': sim_time,
        #     'imu': {
        #         'gyro': [angular_vel['x'], angular_vel['y'], angular_vel['z']],
        #         'accel_body': [linear_accel['x'], linear_accel['y'], linear_accel['z']]
        #     },
        #     'position': position,
        #     'quaternion': quaternion,
        #     'velocity': velocity
        # }
        
        self.test_sim_time += 0.001
        state = {
            "timestamp": round(self.test_sim_time, 3),
            "imu": {
                "gyro": [
                    4.4028248386528135e-12,
                    5.46182226507895e-12,
                    5.454422342398644e-18
                ],
                "accel_body": [
                    -1.521417774123255e-9,
                    1.976595291657851e-9,
                    -9.80000000000001
                ]
            },
            "position": [
                -6.874587183616472e-12,
                -1.6699334495870352e-12,
                -0.1949994162458527
            ],
            "quaternion": [
                1,
                -4.281847537232883e-12,
                1.7627199589076353e-11,
                1.1102230246266445e-15
            ],
            "velocity": [
                -4.862654899757611e-12,
                -9.789131082621268e-13,
                -0.000005569740608881921
            ]
        }

        # json_str = json.dumps(state, indent=0)
        json_str = json.dumps(state, indent=None)

        return json_str
        

    def send_state(self):
        # json_str = self.create_state_json()
        print("Send_State...")

        if self.motor_control_sock:
            # pprint(f"timestamp: {state_json['timestamp']}")
            # import pdb;pdb.set_trace()
            data_json = "\n" + self.state_json + "\n"
            data_json = data_json.encode('utf-8')


            bytes_sent = self.motor_control_sock.sendto(
                data_json,
                (self.fcu_address, self.fcu_port_out)
            )
            print(f"Sent {bytes_sent} bytes to {self.fcu_address}:{self.fcu_port_out}")

    def unpack_servo_packet(self, data):
        # Ensure the data length is valid
        if len(data) != self.SERVO_PACKET_SIZE:
            raise ValueError(f"Data length must be {self.SERVO_PACKET_SIZE} bytes")
        
        # Define the format string according to the structure for little-endian
        # '<HHI16H' specifies:
        #   - < (little-endian byte order)
        #   - HH (2 uint16_t fields)
        #   - I (1 uint32_t field)
        #   - 16H (16 uint16_t fields for pwm values)
        format_string = '<HHI16H'
        
        # Unpack the data
        pkt_magic, pkt_frame_rate, pkt_frame_count, *pkt_pwm = struct.unpack(format_string, data)

        if pkt_magic != self.SERVO_PACKET_MAGIC:
            print(f"Incorrect protocol magic {pkt_magic}, should be {self.SERVO_PACKET_MAGIC}")
            pkt_magic, pkt_frame_rate, pkt_frame_count, pkt_pwm = None, None, None, None
        
        return pkt_magic, pkt_frame_rate, pkt_frame_count, pkt_pwm

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

    
    # def receive_servo_packet(self):
    #     wait_ms =  10 if self.arduPilotOnline else 1
    #     wait_sec = wait_ms / 1000

    #     pkt_magic = 0
    #     pkt_frame_rate = 0
    #     pkt_frame_count = 0
    #     pkt_pwm = [0] * 16  # Support only 16 channels

    #     try:
    #         # Set socket timeout based on waitMs
    #         self.motor_control_sock.settimeout(wait_sec)

    #         # Receiving the data
    #         # Getting FCU (Arudpilot SITL) address and port here.
    #         data, (client_addr, client_out) = self.motor_control_sock.recvfrom(self.SERVO_PACKET_SIZE)
            
    #         if self.fcu_address is None or self.fcu_port_out is None:
    #                 self.fcu_address = client_addr
    #                 self.fcu_port_out = client_out

    #         pkt_magic, pkt_frame_rate, pkt_frame_count, *pkt_pwm = self.unpack_servo_packet(data)
    #         print(f"Received packet: magic={pkt_magic}, frame_rate={pkt_frame_rate}, frame_count={pkt_frame_count}, pwm={pkt_pwm}")

    #     except socket.timeout:
    #         print("Socket timeout")

    #         if self.arduPilotOnline:
    #             self.connectionTimeoutCount += 1
    #             if self.connectionTimeoutCount > self.connectionTimeoutMaxCount:
    #                 self.connectionTimeoutCount = 0
    #                 if self.isLockStep:
    #                     print("Send State from RecieveServoPacket")
    #                     self.send_state()
    #                 else:
    #                     self.arduPilotOnline = False
    #                     print(f"Broken ArduPilot connection, resetting motor control.")
    #                     # self.reset_pids()
    #         # return False

    #     # Check for magic number
    #     magic_16 = 18458
    #     if pkt_magic != magic_16:
    #         print(f"Incorrect protocol magic {pkt_magic}, should be {magic_16}")
    #         return False
    #     print("Magic OK")

    #     # Handle controller online status
    #     if not self.arduPilotOnline:
    #         self.arduPilotOnline = True
    #         print(f"Connected to ArduPilot controller @ {self.fdm_address}:{self.fdm_port_in}")
    #     print("arduPilotOnline")

    #     # Update frame rate
    #     self.fdm_frame_rate = pkt_frame_rate

    #     # Check for controller reset
    #     print("pkt_frame_count < self.fdm_frame_count")
    #     if pkt_frame_count < self.fdm_frame_count:
    #         print("ArduPilot controller has reset")
    #     elif pkt_frame_count == self.fdm_frame_count:
    #         print("Duplicate input frame")
    #         if self.isLockStep:
    #             print("Send State from RecieveServoPacket 2")
    #             self.send_state()
    #         return False
    #     elif pkt_frame_count != self.fdm_frame_count + 1 and self.arduPilotOnline:
    #         print(f"Missed {pkt_frame_count - self.fdm_frame_count} input frames")

    #     # Update frame count
    #     self.fdm_frame_count = pkt_frame_count

    #     # Reset the connection timeout count
    #     self.connectionTimeoutCount = 0

    #     # Update motor commands with the received PWM values
    #     # self.update_motor_commands(pkt_pwm)

    def receive_servo_packet(self):
        # Determine wait time based on whether ArduPilot is online
        # wait_ms = 10 if self.arduPilotOnline else 1
        wait_ms = 1
        wait_sec = wait_ms / 1000

        pkt_frame_rate = 0
        pkt_frame_count = 0

        try:
            # Set socket timeout based on wait_ms
            self.motor_control_sock.settimeout(wait_sec)

            # Receiving the data and getting the client address and port
            data, (client_addr, client_out) = self.motor_control_sock.recvfrom(self.SERVO_PACKET_SIZE)

            # Store the FCU (ArduPilot SITL) address and port if not already set
            if self.fcu_address is None or self.fcu_port_out is None:
                self.fcu_address = client_addr
                self.fcu_port_out = client_out

            # Unpack the received packet
            pkt_magic, pkt_frame_rate, pkt_frame_count, *pkt_pwm = self.unpack_servo_packet(data)
            print(f"Received packet: magic={pkt_magic}, frame_rate={pkt_frame_rate}, frame_count={pkt_frame_count}, pwm={pkt_pwm}")

        except socket.timeout:
            if self.arduPilotOnline:
                self.connectionTimeoutCount += 1
                if self.connectionTimeoutCount > self.connectionTimeoutMaxCount:
                    self.connectionTimeoutCount = 0
                    if self.isLockStep:
                        print("Send State from receive_servo_packet")
                        self.send_state()
                    # else:
                    print("Socket timeout")
                    self.arduPilotOnline = False
                    print(f"Broken ArduPilot connection, resetting motor control.")
                    # Reset PID controllers or other necessary components
            return False
        
        # Handle ArduPilot online status
        if not self.arduPilotOnline:
            print(f"Connected to ArduPilot controller @ {self.fdm_address}:{self.fdm_port_in}")
            # return False
            self.arduPilotOnline = True

        # Update frame rate
        self.fdm_frame_rate = pkt_frame_rate

        # Check for controller reset or duplicate frames
        if pkt_frame_count < self.fdm_frame_count:
            print("ArduPilot controller has reset")
        elif pkt_frame_count == self.fdm_frame_count:
            print("Duplicate input frame")
            if self.isLockStep:
                print("Send State from receive_servo_packet 2")
                # state_json = self.create_state_json(self.last_controller_update_time) # TODO
                self.send_state()
            # return False # TODO
        elif pkt_frame_count != self.fdm_frame_count + 1 and self.arduPilotOnline:
            print(f"Missed {pkt_frame_count - self.fdm_frame_count} input frames")

        # Update frame count
        self.fdm_frame_count = pkt_frame_count

        # Reset the connection timeout count
        self.connectionTimeoutCount = 0

        # Update motor commands with the received PWM values
        # self.update_motor_commands(pkt_pwm)

        return True
    
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
    
    
    def pre_update(self):
        # Update the control surfaces
        
        recieved = self.receive_servo_packet()
        if recieved:
            self.last_servo_packet_recv_time = self.get_sim_duration()

        return recieved
    
        # if self.dataPtr.arduPilotOnline:
        #     dt = (_info['simTime'] - self.dataPtr.lastControllerUpdateTime).total_seconds()
        #     self.apply_motor_forces(dt, _ecm)

    def post_update(self, sensor_data=None): # TODO sensor_data
        if self.sim_time > self.last_controller_update_time and self.arduPilotOnline:
            self.last_controller_update_time = self.get_sim_duration()
            self.state_json = self.create_state_json(self.last_controller_update_time, sensor_data)
            self.send_state()


if __name__ == '__main__':

    # ap = ArduPilotPlugin()
    # ap.drain_unread_packets()
    # ap.motor_control_sock.close()
    # exit(1)

    ap = ArduPilotPlugin()
    ap.drain_unread_packets()

    while True:
        ap.pre_update()
        ap.post_update()

        time.sleep(0.1)
        









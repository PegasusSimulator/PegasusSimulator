
# OLD SNIPETS OF CODE THAT WORK

# -------------------------------------
# Adding a robot to the stage
# -------------------------------------

# Tell this stage that a USD model of a drone exists and where it is "inside the file"

#add_reference_to_stage(usd_path=ROBOTS["Quadrotor"], prim_path="/World/quadrotor")

# Create the a "robot" object wrapper around the "/World/quadrotor" primitive
#self.robot = Robot(
#   prim_path="/World/quadrotor",
#   position=np.array([0.0, 0.0, 1.0]),
#   articulation_controller=None)

# Add the drone USD model to the scene
#self._world.scene.add(self.robot)

#self._world.scene.add()

# def send_sensors(self, time_usec, imu_data, mag_data, baro_data):
#         """
#         The IMU, MAG and BAROMETER readings in SI units in NED body frame
#         time_usec                 : Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
#         --------
#         IMU_DATA
#         --------
#         xacc                      : X acceleration [m/s/s] (type:float)
#         yacc                      : Y acceleration [m/s/s] (type:float)
#         zacc                      : Z acceleration [m/s/s] (type:float)
#         xgyro                     : Angular speed around X axis in body frame [rad/s] (type:float)
#         ygyro                     : Angular speed around Y axis in body frame [rad/s] (type:float)
#         zgyro                     : Angular speed around Z axis in body frame [rad/s] (type:float)
#         --------
#         MAG_DATA
#         --------
#         xmag                      : X Magnetic field [gauss] (type:float)
#         ymag                      : Y Magnetic field [gauss] (type:float)
#         zmag                      : Z Magnetic field [gauss] (type:float)
#         ---------
#         BARO_DATA
#         ---------
#         abs_pressure              : Absolute pressure [hPa] (type:float)
#         diff_pressure             : Differential pressure (airspeed) [hPa] (type:float)
#         pressure_alt              : Altitude calculated from pressure (type:float)
#         temperature               : Temperature [degC] (type:float)
#         ---------
#         fields_updated            : Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim. (type:uint32_t)
#         id                        : Sensor ID (zero indexed). Used for multiple sensor inputs (type:uint8_t)
#         """

#         # Get the accelerometer and gyro data from the simulated imu
#         # converted from ENU to NED convention
#         xgyro =  imu_data["angular_velocity"][0]
#         ygyro = -imu_data["angular_velocity"][1]
#         zgyro = -imu_data["angular_velocity"][2]

#         # Get the magnetic field from the simulated magnetometer


#         # Get the pressure and temperature from simulated barometer

#         # Setup the update code for the mavlink message 

#         #self._connection.mav.hil_sensor_send(
#         #    time_usec, 
#         #    )

#     def send_vision_position(self, time_usec, vision_mocap_data):
#         """
#         This is usefull if simulating a visual inertial plugin running inside Isaac SIM (maybe in the future), or
#         to just simulate sending the data that a MOCAP system would send to the vehicle for fusion
#         """

#         # TODO - implement this feature and handle the ENU to NED convention required by PX4
#         x = 0.0         # Local X global position in inertial frame [m] (type:float)
#         y = 0.0         # Local Y global position in inertial frame [m] (type:float)
#         z = 0.0         # Local Z global position in inertial frame [m] (type:float)
#         roll = 0.0      # Roll angle [rad] (type:float)
#         pitch = 0.0     # Pitch angle [rad] (type:float)
#         yaw = 0.0       # Yaw angle [rad] (type:float)

#         # Row-major representation of pose 6x6 cross-covariance matrix upper right triangle 
#         # (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries 
#         # are the second ROW, etc.). If unknown, assign NaN value to first element in the array. (type:float)
#         covariance=(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        
#         # Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions 
#         # (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system 
#         # detects a loop-closure and the estimate jumps. (type:uint8_t)
#         reset_counter = 0

#         # Note: to know more about these functions, go to pymavlink->dialects->v20->standard.py
#         # This contains the definitions for sending the hearbeat and simulated sensor messages
#         self._connection.mav.global_vision_position_estimate_send(
#             time_usec,        # Timestamp (UNIX time or time since system boot) [us] (type:uint64_t)
#             x, y, z,
#             roll, pitch, yaw,
#             covariance,   
#             reset_counter)

#     def send_sim_data(self, time_usec, imu_data, mag_data, baro_data, gps_data):
#         """
#         Status of simulation environment

#         q1, q2, q3, q4             : True attitude quaternion components, w, x, y, z (type:float)
#         roll, pitch, yaw           : Attitude roll, pitch, yaw expressed as Euler angles
#         xacc, yacc, zacc           : X, Y, Z acceleration [m/s/s] (type:float)
#         xgyro, ygyro, zgyro        : Angular speed around X, Y, Z axis [rad/s] (type:float)
#         lat, lon, alt              : Latitude [deg], Longitude [deg], Altitude [m] (type:float)
#         std_dev_horz, std_dev_vert : Horizontal/Vertical position standard deviation (type:float)
#         vn, ve, vd                 : True velocity in north, east, down direction in earth-fixed NED frame [m/s] (type:float)
#         ve                        : True velocity in east direction in earth-fixed NED frame [m/s] (type:float)
#         vd                        : True velocity in down direction in earth-fixed NED frame [m/s] (type:float)

#         """
#         # TODO
        
#         # Quaternion with the orientation of the vehicle
#         q1 = 1.0
#         q2 = 0.0
#         q3 = 0.0
#         q4 = 0.0

#         # Euler angles with the orientation of the vehicle
#         roll = 0.0
#         pitch = 0.0
#         yaw = 0.0

#         # Linear acceleration measured by the im
#         xacc = 0.0
#         yacc = 0.0
#         zacc = 0.0

#         # Angular velocity measured by the imu
#         # converted from ENU to NED convention
#         xgyro =  imu_data["angular_velocity"][0]
#         ygyro = -imu_data["angular_velocity"][1]
#         zgyro = -imu_data["angular_velocity"][2]

#         # Get the latitude and longitude from the GPS data 
#         lat = np.degrees(gps_data["latitude"])
#         lon = np.degrees(gps_data["longitude"])
#         alt = gps_data["altitude"]

#         # Set the standard deviation to a fixed value (zero for now - fix later)
#         std_dev_horz = 0
#         std_dev_vert = 0

#         # Get the velocity measured by the GPS
#         vn = gps_data["velocity_north"]
#         ve = gps_data["velocity_east"]
#         vd = gps_data["velocity_down"]

#         self._connection.mav.sim_state_send(
#             q1, q2, q3, q4,                 # w, x, y, z convention
#             roll, pitch, yaw,               # rad
#             xacc, yacc, zacc,               # m/s^2
#             xgyro, ygyro, zgyro,            # rad/s
#             lat, lon, alt,                  # [deg, deg, m]
#             std_dev_horz, std_dev_vert,     # Horizontal/Vertical position standard deviation
#             vn, ve, vd                      # Velocity north, east, down m/s
#         )


# Get the latitude and longitude from the current state
    latitude, longitude = reprojection(state.position, self._origin_latitude, self._origin_longitude)

    # Magnetic declination and inclination (radians)
    declination_rad: float = np.radians(get_mag_declination(np.degrees(latitude), np.degrees(longitude)))
    inclination_rad: float = np.radians(get_mag_inclination(np.degrees(latitude), np.degrees(longitude)))
    
    # Compute the magnetic strength (10^5xnanoTesla)
    strength_ga: float = 0.01 * get_mag_strength(np.degrees(latitude), np.degrees(longitude))
    
    # Compute the Magnetic filed components according to: http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
    H: float = strength_ga * np.cos(inclination_rad)
    Z: float = np.tan(inclination_rad) * H
    X: float = H * np.cos(declination_rad)
    Y: float = H * np.sin(declination_rad)
    
    # Magnetic field of a body following a front-left-up (FLU) convention expressed in a East-North-Up (ENU) inertial frame
    magnetic_field_inertial: np.ndarray = np.array([X, Y, Z])
    
    # Rotate the magnetic field vector such that it expresses a field of a body frame according to the front-right-down (FRD)
    # expressed in a North-East-Down (NED) inertial frame (the standard used in magnetometer units)
    attitude_flu_enu = Rotation.from_quat(state.attitude)

    # Rotate the magnetic field from the inertial frame to the body frame of reference according to the FLU frame convention
    
    q_body_to_world = rot_ENU_to_NED * attitude_flu_enu * rot_FLU_to_FRD.inv()
    magnetic_field_B = q_body_to_world.inv().apply(q_body_to_world.apply(magnetic_field_inertial))
    magnetic_field_body = [magnetic_field_B[1], -magnetic_field_B[0], magnetic_field_B[2]]

    carb.log_warn("NOT THIS")
    
    #magnetic_field_body: np.ndarray = attitude_flu_enu.inv().apply(magnetic_field_inertial)
    #magnetic_field_body[2] *= -1

    #rot_body_to_world = rot_ENU_to_NED * attitude_flu_enu * rot_test.inv()

    # The magnetic field expressed in the body frame according to the front-right-down (FRD) convention
    #magnetic_field_body = rot_body_to_world.apply(magnetic_field_inertial)
    
    # -------------------------------
    # Add noise to the magnetic field
    # -------------------------------
    tau = self._bias_correlation_time
    
    # Discrete-time standard deviation equivalent to an "integrating" sampler with integration time dt.
    sigma_d: float = 1 / np.sqrt(dt) * self._noise_density
    sigma_b: float = self._random_walk
    
    # Compute exact covariance of the process after dt [Maybeck 4-114].
    sigma_b_d: float = np.sqrt( - sigma_b * sigma_b * tau / 2.0 * (np.exp(-2.0 * dt / tau) - 1.0))
    
    # Compute state-transition.
    phi_d: float = np.exp(-1.0 / tau * dt)
    
    # Add the noise to the magnetic field
    magnetic_field_noisy: np.ndarray = np.zeros((3,))
    for i in range(3):
        self._bias[i] = phi_d * self._bias[i] + sigma_b_d * np.random.randn()
        magnetic_field_noisy[i] = magnetic_field_body[i] + sigma_d * np.random.randn() #+ self._bias[i]

    # Add the values to the dictionary and return it
    self._state = {'magnetic_field': magnetic_field_noisy}

    return self._state
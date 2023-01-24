#!/usr/bin/env python

import carb
import numpy as np
from pegasus_isaac.logic.vehicles.vehicle import Vehicle

# Mavlink interface
from pegasus_isaac.logic.backends.mavlink_interface import MavlinkInterface

# Sensors and dynamics setup
from pegasus_isaac.logic.sensors import Barometer, IMU, Magnetometer, GPS
from pegasus_isaac.logic.dynamics import LinearDrag, QuadraticThrustCurve

class MultirotorConfig:

    def __init__(self):

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "quadrotor"

        # The USD file that describes the visual aspect of the vehicle (and some properties such as mass and moments of inertia)
        self.udf_file = ""

        # The default thrust curve for a quadrotor
        self.thrust_curves = QuadraticThrustCurve()

        # The default sensors for a quadrotor
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]

        # The backend for the mavlink of this vehicle (with default mavlink configurations) 
        # [Can be None as well, if we do not desired to use PX4 with this simulated vehicle]
        self.mavlink_backend = MavlinkInterface()

        # The backend for the ROS2 interface of this vehicle
        self.ros2_backend = None

class Multirotor(Vehicle):

    def __init__(
        self, 
        # Simulation specific configurations
        stage_prefix: str="quadrotor",  
        usd_file: str="",
        world=None,
        # Spawning pose of the vehicle
        init_pos=[0.0, 0.0, 0.07], 
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        lat=47.397742,
        long=8.545594,
        alt=488.0
    ):

        # Create a mavlink interface for getting data on the desired port. If it fails, do not spawn the vehicle
        # on the simulation world and just throw an exception
        try:
            self._mavlink = MavlinkInterface()
        except Exception as e:
            carb.log_error("Could not initiate the mavlink interface. Not spawning the vehicle. Full error log: ")
            carb.log_error(e)
        
        # Initiate the Vehicle
        super().__init__(stage_prefix, usd_file, world, init_pos, init_orientation)

        # Create a Quadratic Thrust curve for representing the thrusters
        self._thrusters = QuadraticThrustCurve()

        # Create the sensors that a quadrotor typically has
        self._barometer = Barometer(altitude_home=alt)
        self._imu = IMU()
        self._magnetometer = Magnetometer(lat, long)
        self._gps = GPS(lat, long, origin_altitude=alt)
        self._linear_drag = LinearDrag(np.array([0.50, 0.30, 0.0]))
        
        # Add callbacks to the physics engine to update the sensors every timestep
        self._world.add_physics_callback(self._stage_prefix + "/barometer", self.update_barometer_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/imu", self.update_imu_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/magnetometer", self.update_magnetometer_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/gps", self.update_gps_sensor)
        self._world.add_physics_callback(self._stage_prefix + "/mav_state", self.update_sim_state_mav)

        # Add a callback to start/stop the mavlink streaming once the play/stop button is hit
        self._world.add_timeline_callback(self._stage_prefix + "/start_stop_sim", self.sim_start_stop)

        self.total_time = 0

    def update_barometer_sensor(self, dt: float):
        sensor_data = self._barometer.update(self._state, dt)
        if sensor_data is not None:
            self._mavlink.update_bar_data(sensor_data)

    def update_imu_sensor(self, dt: float):
        sensor_data = self._imu.update(self._state, dt)
        if sensor_data is not None:
            self._mavlink.update_imu_data(sensor_data)

    def update_magnetometer_sensor(self, dt: float):
        sensor_data = self._magnetometer.update(self._state, dt)
        if sensor_data is not None:
            self._mavlink.update_mag_data(sensor_data)

    def update_gps_sensor(self, dt: float):
        sensor_data = self._gps.update(self._state, dt)
        if sensor_data is not None:
            self._mavlink.update_gps_data(sensor_data)

    def update_sim_state_mav(self, dt: float): 
        self._mavlink.update_sim_state(self._state)

    def sim_start_stop(self, event):
        """
        Callback that is called every time there is a timeline event such as starting/stoping the simulation
        """
        
        # If the start/stop button was pressed, then start/stop mavlink communication
        if self._world.is_playing():
            self._mavlink.start_stream()
            pass

        if self._world.is_stopped():
            self._mavlink.stop_stream()
            pass

    def update(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type
        """

        # Get the articulation root of the vehicle
        articulation = self._world.dc_interface.get_articulation(self._stage_prefix  + "/vehicle/body")

        # Get the desired angular velocities for each rotor from the backend (can be mavlink or other) expressed in rad/s
        desired_rotor_velocities = self._mavlink._rotor_data.input_reference

        # Input the desired rotor velocities in the thruster model
        self._thrusters.set_input_reference(desired_rotor_velocities)

        # Get the desired forces to apply to the vehicle
        forces_z, _, rolling_moment = self._thrusters.update(self._state, dt)
        
        # Apply force to each rotor
        for i in range(4):

            # Apply the force in Z on the rotor frame
            self.apply_force([0.0, 0.0, forces_z[i]], body_part="/rotor" + str(i))

            # Generate the rotating propeller visual effect
            self.handle_propeller_visual(i, forces_z[i], articulation)

        # Apply the torque to the body frame of the vehicle that corresponds to the rolling moment
        self.apply_torque([0.0, 0.0, rolling_moment], "/body")

        # Compute the total linear drag force to apply to the vehicle's body frame
        drag = self._linear_drag.update(self._state, dt)
        self.apply_force(drag, body_part="/body")

        self.total_time += dt
        self._mavlink.mavlink_update(dt)

    def handle_propeller_visual(self, rotor_number, force: float, articulation):

        # Rotate the joint to yield the visual of a rotor spinning (for animation purposes only)
        joint = self._world.dc_interface.find_articulation_dof(articulation, "joint" + str(rotor_number))

        # Spinning when armed but not applying force
        if 0.0 < force < 0.1:
            self._world.dc_interface.set_dof_velocity(joint, 5 * self._thrusters.rot_dir[rotor_number])
        # Spinning when armed and applying force
        elif 0.1 <= force:
            self._world.dc_interface.set_dof_velocity(joint, 100 * self._thrusters.rot_dir[rotor_number])
        # Not spinning    
        else:
            self._world.dc_interface.set_dof_velocity(joint, 0)

#!/usr/bin/env python

import carb
import numpy as np
from pegasus_isaac.logic.vehicles.vehicle import Vehicle

# Mavlink interface
from pegasus_isaac.logic.backends.mavlink_backend import MavlinkBackend

# Sensors and dynamics setup
from pegasus_isaac.logic.sensors import Barometer, IMU, Magnetometer, GPS
from pegasus_isaac.logic.dynamics import LinearDrag, QuadraticThrustCurve

class MultirotorConfig:

    def __init__(self):

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "quadrotor"

        # The USD file that describes the visual aspect of the vehicle (and some properties such as mass and moments of inertia)
        self.usd_file = ""

        # The default thrust curve for a quadrotor and dynamics relating to drag
        self.thrust_curve = QuadraticThrustCurve()
        self.drag = LinearDrag(np.array([0.50, 0.30, 0.0]))

        # The default sensors for a quadrotor
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]

        # The backends for actually sending commands to the vehicle. By default use mavlink (with default mavlink configurations) 
        # [Can be None as well, if we do not desired to use PX4 with this simulated vehicle]. It can also be a ROS2 backend
        # or your own custom Backend implementation! 
        self.backends = [MavlinkBackend()]

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
        alt=488.0,
        config=MultirotorConfig()
    ):
        
        # 1. Initiate the Vehicle object itself
        super().__init__(stage_prefix, usd_file, world, init_pos, init_orientation)

        # 2. Initialize all the vehicle sensors
        self._sensors = config.sensors
        for sensor in self._sensors:
            sensor.initialize(lat, long, alt)

        # Add callbacks to the physics engine to update each sensor at every timestep
        # and let the sensor decide depending on its internal update rate whether to generate new data
        self._world.add_physics_callback(self._stage_prefix + "/Sensors", self.update_sensors)

        # 3. Setup the dynamics of the system
        # Get the thrust curve of the vehicle from the configuration
        self._thrusters = config.thrust_curve
        self._linear_drag = config.drag

        # 4. Save the backend interface (if given in the configuration of the multirotor)
        self._backends = config.backends
        
        # Add a callbacks for the 
        self._world.add_physics_callback(self._stage_prefix + "/mav_state", self.update_sim_state)

    def update_sensors(self, dt: float):

        # Call the update method for the sensor to update its values internally (if applicable)
        for sensor in self._sensors:
            sensor_data = sensor.update(self._state, dt)
        
            # If some data was updated and we have a mavlink backend or ros backend (or other), then just update it
            if sensor_data is not None:
                for backend in self._backends:
                    backend.update_sensor(sensor.sensor_type, sensor_data)

    def update_sim_state(self, dt: float): 
        """
        Callback that is used to "send" the current state for each backend being used to control the vehicle
        """
        for backend in self._backends:
            backend.update_state(self._state)

    def start(self):
        # Intialize the communication with all the backends
        for backend in self._backends:
            backend.start()

    def stop(self):
        # Signal all the backends that the simulation has stoped
        for backend in self._backends:
            backend.stop()

    def update(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type
        """

        # Get the articulation root of the vehicle
        articulation = self._world.dc_interface.get_articulation(self._stage_prefix  + "/vehicle/body")

        # Get the desired angular velocities for each rotor from the first backend (can be mavlink or other) expressed in rad/s
        # For now we are only getting the desired inputs from the first backend. TODO - add a more dynamic way of getting
        # the controls of the vehicles from multiple places to allow overriding of controls - That would be way coooler
        if len(self._backends) != 0:
            desired_rotor_velocities = self._backends[0].input_reference()

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

        # Call the update methods in all backends
        for backend in self._backends:
            backend.update(dt)

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

"""
| File: vehicle.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Vehicle class which is used as the base for all the vehicles.
"""

# Numerical computations
import numpy as np
from scipy.spatial.transform import Rotation

# Low level APIs
import carb
from pxr import Usd, Gf

# High level Isaac sim APIs
import omni.usd
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.usd import get_stage_next_free_path
from omni.isaac.core.robots.robot import Robot
from omni.isaac.dynamic_control import _dynamic_control

# Extension APIs
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicle_manager import VehicleManager


def get_world_transform_xform(prim: Usd.Prim):
    """
    Get the local transformation of a prim using omni.usd.get_world_transform_matrix().
    See https://docs.omniverse.nvidia.com/kit/docs/omni.usd/latest/omni.usd/omni.usd.get_world_transform_matrix.html
    Args:
        prim (Usd.Prim): The prim to calculate the world transformation.
    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
        - Scale vector.
    """
    world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
    rotation: Gf.Rotation = world_transform.ExtractRotation()
    return rotation


class Vehicle(Robot):
    
    def __init__(
        self,
        stage_prefix: str,
        usd_path: str = None,
        init_pos=[0.0, 0.0, 0.0],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        sensors=[],
        graphical_sensors=[],
        backends=[]
    ):
        """
        Class that initializes a vehicle in the isaac sim's curent stage

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. Defaults to "quadrotor".
            usd_path (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            init_pos (list): The initial position of the vehicle in the inertial frame (in ENU convention). Defaults to [0.0, 0.0, 0.0].
            init_orientation (list): The initial orientation of the vehicle in quaternion [qx, qy, qz, qw]. Defaults to [0.0, 0.0, 0.0, 1.0].
        """

        # Get the current world at which we want to spawn the vehicle
        self._world = PegasusInterface().world
        self._current_stage = self._world.stage

        # Save the name with which the vehicle will appear in the stage
        # and the name of the .usd file that contains its description
        self._stage_prefix = get_stage_next_free_path(self._current_stage, stage_prefix, False)
        self._usd_file = usd_path

        # Get the vehicle name by taking the last part of vehicle stage prefix
        self._vehicle_name = self._stage_prefix.rpartition("/")[-1]

        # Spawn the vehicle primitive in the world's stage
        self._prim = define_prim(self._stage_prefix, "Xform")
        self._prim = get_prim_at_path(self._stage_prefix)
        self._prim.GetReferences().AddReference(self._usd_file)

        # Initialize the "Robot" class
        # Note: we need to change the rotation to have qw first, because NVidia
        # does not keep a standard of quaternions inside its own libraries (not good, but okay)
        super().__init__(
            prim_path=self._stage_prefix,
            name=self._stage_prefix,
            position=init_pos,
            orientation=[init_orientation[3], init_orientation[0], init_orientation[1], init_orientation[2]],
            articulation_controller=None,
        )

        self._vehicle_dc_interface = None

        # Add this object for the world to track, so that if we clear the world, this object is deleted from memory and
        # as a consequence, from the VehicleManager as well
        self._world.scene.add(self)

        # Add the current vehicle to the vehicle manager, so that it knows
        # that a vehicle was instantiated
        VehicleManager.get_vehicle_manager().add_vehicle(self._stage_prefix, self)

        # Variable that will hold the current state of the vehicle
        self._state = State()

        # Add a callback to the physics engine to update the current state of the system
        self._world.add_physics_callback(self._stage_prefix + "/state", self.update_state)

        # Add the update method to the physics callback if the world was received
        # so that we can apply forces and torques to the vehicle. Note, this method should
        # be implemented in classes that inherit the vehicle object
        self._world.add_physics_callback(self._stage_prefix + "/update", self.update)

        # Set the flag that signals if the simulation is running or not
        self._sim_running = False

        # Add a callback to start/stop of the simulation once the play/stop button is hit
        self._world.add_timeline_callback(self._stage_prefix + "/start_stop_sim", self.sim_start_stop)

        # --------------------------------------------------------------------
        # -------------------- Add sensors to the vehicle --------------------
        # --------------------------------------------------------------------
        self._sensors = sensors
        
        for sensor in self._sensors:
            sensor.initialize(self, PegasusInterface().latitude, PegasusInterface().longitude, PegasusInterface().altitude)

        # Add callbacks to the physics engine to update each sensor at every timestep
        # and let the sensor decide depending on its internal update rate whether to generate new data
        self._world.add_physics_callback(self._stage_prefix + "/Sensors", self.update_sensors)

        # --------------------------------------------------------------------
        # -------------------- Add the graphical sensors to the vehicle ------
        # --------------------------------------------------------------------
        self._graphical_sensors = graphical_sensors

        for graphical_sensor in self._graphical_sensors:
            graphical_sensor.initialize(self)

        # Add callbacks to the rendering engine to update each graphical sensor at every timestep of the rendering engine
        self._world.add_render_callback(self._stage_prefix + "/GraphicalSensors", self.update_graphical_sensors)
        
        # --------------------------------------------------------------------
        # ---- Add (communication/control) backends to the vehicle -----------
        # --------------------------------------------------------------------
        self._backends = backends

        # Initialize the backends
        for backend in self._backends:
            backend.initialize(self)

        # Add a callbacks for the
        self._world.add_physics_callback(self._stage_prefix + "/mav_state", self.update_sim_state)


    def __del__(self):
        """
        Method that is invoked when a vehicle object gets destroyed. When this happens, we also invoke the 
        'remove_vehicle' from the VehicleManager in order to remove the vehicle from the list of active vehicles.
        """

        # Remove this object from the vehicleHandler
        VehicleManager.get_vehicle_manager().remove_vehicle(self._stage_prefix)

    """
    Properties
    """

    @property
    def state(self):
        """The state of the vehicle.

        Returns:
            State: The current state of the vehicle, i.e., position, orientation, linear and angular velocities...
        """
        return self._state
    
    @property
    def vehicle_name(self) -> str:
        """Vehicle name.

        Returns:
            Vehicle name (str): last prim name in vehicle prim path
        """
        return self._vehicle_name

    """
    Operations
    """

    def sim_start_stop(self, event):
        """
        Callback that is called every time there is a timeline event such as starting/stoping the simulation.

        Args:
            event: A timeline event generated from Isaac Sim, such as starting or stoping the simulation.
        """

        # If the start/stop button was pressed, then call the start and stop methods accordingly
        if self._world.is_playing() and self._sim_running == False:
            self._sim_running = True

            # Initialize the sensors
            for sensor in self._sensors:
                sensor.start()

            # Initialize the graphical sensors
            for graphical_sensor in self._graphical_sensors:
                graphical_sensor.start()

            # Intializes the communication with all the backends. This method is invoked automatically when the simulation starts
            for backend in self._backends:
                backend.start()

            # Invoke the start method of the vehicle (if it exists)
            self.start()

        if self._world.is_stopped() and self._sim_running == True:
            self._sim_running = False

            # Reset the DC interface
            self._vehicle_dc_interface = None

            # Stop the sensors
            for sensor in self._sensors:
                sensor.stop()

            # Stop the graphical sensors
            for graphical_sensor in self._graphical_sensors:
                graphical_sensor.stop

            # Signal all the backends that the simulation has stoped. This method is invoked automatically when the simulation stops
            for backend in self._backends:
                backend.stop()

            self.stop()

    def apply_force(self, force, pos=[0.0, 0.0, 0.0], body_part="/body"):
        """
        Method that will apply a force on the rigidbody, on the part specified in the 'body_part' at its relative position
        given by 'pos' (following a FLU) convention. 

        Args:
            force (list): A 3-dimensional vector of floats with the force [Fx, Fy, Fz] on the body axis of the vehicle according to a FLU convention.
            pos (list): _description_. Defaults to [0.0, 0.0, 0.0].
            body_part (str): . Defaults to "/body".
        """

        # Get the handle of the rigidbody that we will apply the force to
        rb = self.get_dc_interface().get_rigid_body(self._stage_prefix + body_part)

        # Apply the force to the rigidbody. The force should be expressed in the rigidbody frame
        self.get_dc_interface().apply_body_force(rb, carb._carb.Float3(force), carb._carb.Float3(pos), False)

    def apply_torque(self, torque, body_part="/body"):
        """
        Method that when invoked applies a given torque vector to /<rigid_body_name>/"body" or to /<rigid_body_name>/<body_part>.

        Args:
            torque (list): A 3-dimensional vector of floats with the force [Tx, Ty, Tz] on the body axis of the vehicle according to a FLU convention.
            body_part (str): . Defaults to "/body".
        """

        # Get the handle of the rigidbody that we will apply a torque to
        rb = self.get_dc_interface().get_rigid_body(self._stage_prefix + body_part)

        # Apply the torque to the rigidbody. The torque should be expressed in the rigidbody frame
        self.get_dc_interface().apply_body_torque(rb, carb._carb.Float3(torque), False)

    def update_state(self, dt: float):
        """
        Method that is called at every physics step to retrieve and update the current state of the vehicle, i.e., get
        the current position, orientation, linear and angular velocities and acceleration of the vehicle.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Get the body frame interface of the vehicle (this will be the frame used to get the position, orientation, etc.)
        body = self.get_dc_interface().get_rigid_body(self._stage_prefix + "/body")

        # Get the current position and orientation in the inertial frame
        pose = self.get_dc_interface().get_rigid_body_pose(body)

        # Get the attitude according to the convention [w, x, y, z]
        prim = self._world.stage.GetPrimAtPath(self._stage_prefix + "/body")
        rotation_quat = get_world_transform_xform(prim).GetQuaternion()
        rotation_quat_real = rotation_quat.GetReal()
        rotation_quat_img = rotation_quat.GetImaginary()

        # Get the angular velocity of the vehicle expressed in the body frame of reference
        ang_vel = self.get_dc_interface().get_rigid_body_angular_velocity(body)

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        linear_vel = self.get_dc_interface().get_rigid_body_linear_velocity(body)

        # Get the linear acceleration of the body relative to the inertial frame, expressed in the inertial frame
        # Note: we must do this approximation, since the Isaac sim does not output the acceleration of the rigid body directly
        linear_acceleration = (np.array(linear_vel) - self._state.linear_velocity) / dt

        # Update the state variable X = [x,y,z]
        self._state.position = np.array(pose.p)

        # Get the quaternion according in the [qx,qy,qz,qw] standard
        self._state.attitude = np.array(
            [rotation_quat_img[0], rotation_quat_img[1], rotation_quat_img[2], rotation_quat_real]
        )

        # Express the velocity of the vehicle in the inertial frame X_dot = [x_dot, y_dot, z_dot]
        self._state.linear_velocity = np.array(linear_vel)

        # The linear velocity V =[u,v,w] of the vehicle's body frame expressed in the body frame of reference
        # Note that: x_dot = Rot * V
        self._state.linear_body_velocity = (
            Rotation.from_quat(self._state.attitude).inv().apply(self._state.linear_velocity)
        )

        # omega = [p,q,r]
        self._state.angular_velocity = Rotation.from_quat(self._state.attitude).inv().apply(np.array(ang_vel))

        # The acceleration of the vehicle expressed in the inertial frame X_ddot = [x_ddot, y_ddot, z_ddot]
        self._state.linear_acceleration = linear_acceleration

    def start(self):
        """
        Method that should be implemented by the class that inherits the vehicle object.
        """
        pass

    def stop(self):
        """
        Method that should be implemented by the class that inherits the vehicle object.
        """
        pass

    def update(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type and it's called periodically by the physics engine.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        pass

    def update_sensors(self, dt: float):
        """Callback that is called at every physics steps and will call the sensor.update method to generate new
        sensor data. For each data that the sensor generates, the backend.update_sensor method will also be called for
        every backend. For example, if new data is generated for an IMU and we have a MavlinkBackend, then the update_sensor
        method will be called for that backend so that this data can latter be sent thorugh mavlink.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Call the update method for the sensor to update its values internally (if applicable)
        for sensor in self._sensors:
            sensor_data = sensor.update(self._state, dt)

            # If some data was updated and we have a mavlink backend or ros backend (or other), then just update it
            if sensor_data is not None:
                for backend in self._backends:
                    backend.update_sensor(sensor.sensor_type, sensor_data)

    def update_graphical_sensors(self, event):
        """Callback that is called at every rendering steps and will call the graphical_sensor.update method to generate new
        sensor data. For each data that the sensor generates, the backend.update_graphical_sensor method will also be called for
        every backend. For example, if new data is generated for a monocular camera and we have a ROS2Backend, then the update_graphical_sensor
        method will be called for that backend so that this data can latter be sent through a ROS2 topic.

        Args:
            event (float): The timer event that contains the time elapsed between the previous and current function calls (s).
        """

        # Call the update method for the sensor to update its values internally (if applicable)
        for sensor in self._graphical_sensors:
            sensor_data = sensor.update(self._state, event.payload['dt'])

            # If some data was updated and we have a ros backend (or other), then just update it
            if sensor_data is not None:
                for backend in self._backends:
                    backend.update_graphical_sensor(sensor.sensor_type, sensor_data)

    def update_sim_state(self, dt: float):
        """
        Callback that is used to "send" the current state for each backend being used to control the vehicle. This callback
        is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        for backend in self._backends:
            backend.update_state(self._state)

    def get_dc_interface(self):

        if self._vehicle_dc_interface is None:
            self._vehicle_dc_interface = _dynamic_control.acquire_dynamic_control_interface()

        return self._vehicle_dc_interface
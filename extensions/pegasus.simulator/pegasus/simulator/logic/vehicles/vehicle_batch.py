"""
| File: vehicle.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Vehicle class which is used as the base for all the vehicles.
"""
import torch

import math

# Low level APIs
import carb
from pxr import Usd, UsdGeom, Gf

# High level Isaac sim APIs
import omni.usd
from omni.usd import get_stage_next_free_path
import isaacsim.core.utils.stage as stage_utils
#from omni.isaac.dynamic_control import _dynamic_control
from isaacsim.core.prims import RigidPrim

from omni.isaac.cloner import GridCloner
from omni.isaac.core.prims import XFormPrimView

# Extension APIs
from pegasus.simulator.logic.state_batch import StateBatch
from pegasus.simulator.logic.vehicle_manager import VehicleManager
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.transforms import quaternion_apply, quaternion_invert



class VehicleBatch():
    """
    Batch wrapper for multiple vehicles spawned from the same USD asset.

    This class:
    - spawns N vehicles with paths <stage_prefix>0, <stage_prefix>1, ...
    - creates batched RigidPrim views over the spawned rigid bodies
    - stores batched state tensors with shape (N, ...)
    - applies forces and torques to all matched rigid bodies in a single call

    Conventions:
    - stage_prefix must be a base prefix, e.g. "/World/quadrotor"
    - spawned vehicle prims become "/World/quadrotor_0", "/World/quadrotor_1", ...
    - state tensors use shape (n_vehicles, dim)
    """
    def __init__(
        self,
        stage_prefix: str,
        usd_path: str,
        n_vehicles: int = 1,
        init_pos=None,
        init_orientation=None,
        sensors=[],
        graphical_sensors=[],
        graphs=[],
        backends=[],
        spacing: float = 3.0,
):
        """
        Class that initializes a vehicle in the isaac sim's curent stage

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. Defaults to "quadrotor".
            usd_path (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            init_pos (list): The initial position of the vehicle in the inertial frame (in ENU convention). Defaults to [0.0, 0.0, 0.0].
            init_orientation (list): The initial orientation of the vehicle in quaternion [qx, qy, qz, qw]. Defaults to [0.0, 0.0, 0.0, 1.0].
        """

        # Define the same device that is running the simulation
        self.device = PegasusInterface()._world_settings["device"]

        # Get the current world at which we want to spawn the vehicle
        self._world = PegasusInterface().world
        self._stage = self._world.stage
            
        # Save the name with which the vehicle will appear in the stage
        # and the name of the .usd file that contains its description
        self._stage_prefix = get_stage_next_free_path(self._stage, stage_prefix, False)
        self._usd_file = usd_path
        self.n_vehicles = n_vehicles

        self._vehicle_name = self._stage_prefix.rpartition("/")[-1]

        # Spawn the batch of vehicle's in the world's stage
        self._spawn_batch(init_pos, init_orientation, spacing)

        self.parts_per_vehicle = None
        
        self.body_index = None

        # View batch prims
        self._vehicle_expr = f"{self._stage_prefix}.*/.*"
        self.vehicle_prims = RigidPrim(prim_paths_expr=self._vehicle_expr, name=f"{self._stage_prefix}_prims")

        # Variable that will hold the current state of the vehicle
        self._state = StateBatch(self.n_vehicles, self.device)

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
        #self._sensors = sensors
        
        #for sensor in self._sensors:
        #    sensor.initialize(
        #        self, 
        #        torch.tensor(PegasusInterface().latitude, dtype=torch.float32, device=self.device), 
        #        torch.tensor(PegasusInterface().longitude, dtype=torch.float32, device=self.device), 
        #        torch.tensor(PegasusInterface().altitude, dtype=torch.float32, device=self.device)
        #    )

        # Add callbacks to the physics engine to update each sensor at every timestep
        # and let the sensor decide depending on its internal update rate whether to generate new data
        #self._world.add_physics_callback(self._stage_prefix + "/Sensors", self.update_sensors)

        # --------------------------------------------------------------------
        # -------------------- Add the graphical sensors to the vehicle ------
        # --------------------------------------------------------------------
        #self._graphical_sensors = graphical_sensors

        #for graphical_sensor in self._graphical_sensors:
        #    graphical_sensor.initialize(self)

        # Add callbacks to the rendering engine to update each graphical sensor at every timestep of the rendering engine
        #self._world.add_render_callback(self._stage_prefix + "/GraphicalSensors", self.update_graphical_sensors)


        # --------------------------------------------------------------------
        # -------------------- Add the graphs to the vehicle -----------------
        # --------------------------------------------------------------------
        #self._graphs = graphs

        #for graph in self._graphs:
        #    graph.initialize(self)
        
        # --------------------------------------------------------------------
        # ---- Add (communication/control) backends to the vehicle -----------
        # --------------------------------------------------------------------
        self._backends = backends

        #Initialize the backends
        for backend in self._backends:
            backend.initialize(self)

        # Add a callbacks for the
        self._world.add_physics_callback(self._stage_prefix + "/mav_state", self.update_sim_state)


    def initialize(self):
        """
        This method initialize the vehicle prim handles and allocate batched state tensors.
        """
        self.vehicle_prims.initialize()

        self.parts_per_vehicle = self.vehicle_prims.count // self.n_vehicles

        print(f"Spawned {self.n_vehicles} vehicles with {self.parts_per_vehicle} parts each (total {self.vehicle_prims.count} prims)")

        self.body_index = next((i for i, p in enumerate(self.vehicle_prims.prim_paths[:self.parts_per_vehicle]) if p.endswith("/body")), None)

        self._allocate_batch_state()


    def _spawn_batch(self, init_pos=None, init_orientation=None, spacing=3.0):
        '''
        This method spawns a batch of vehicles in the simulation stage.

        If explicit initial positions are provided, each vehicle is spawned individually at the specified position and orientation.
        Otherwise, a GridCloner is used to spawn the vehicles automatically in a grid formation with the given spacing.

        Args:
            spacing (float): Distance between vehicles when using the grid spawn mode.
        '''

        # If explicit initial positions were provided, spawn each vehicle manually
        if init_pos is not None:

            for i in range(self.n_vehicles):
                
                # Create the prim path for this vehicle instance
                prim_path = f"{self._stage_prefix}_{i}"

                # Add the USD reference of the vehicle to the stage
                stage_utils.add_reference_to_stage(self._usd_file, prim_path=prim_path)

                # Get the prim and create XForm interfaces to manipulate its transform
                prim = self._stage.GetPrimAtPath(prim_path)
                xformable = UsdGeom.Xformable(prim)
                xform = UsdGeom.XformCommonAPI(xformable)

                # Set the initial position of the vehicle in world coordinates
                p = init_pos[i]
                xform.SetTranslate(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))

                # If initial orientations were provided, apply them using a quaternion
                if init_orientation is not None:
                    q = init_orientation[i]   # [w, x, y, z]
                    quat = Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3])).GetNormalized()

                    # Check if the prim already has an orient transform op
                    orient_attr = prim.GetAttribute("xformOp:orient")
                    
                    if orient_attr.IsValid():
                        orient_attr.Set(quat)
                    else:
                        orient_op = xformable.AddOrientOp()
                        orient_op.Set(quat)
        
        # If no explicit positions were given, spawn vehicles using GridCloner
        else:

            # Spawn the base vehicle that will be used as the cloning source
            prim_path = f"{self._stage_prefix}_0"
            stage_utils.add_reference_to_stage(self._usd_file, prim_path=prim_path)

            # Create a grid cloner that will distribute vehicles with the given spacing
            cloner = GridCloner(spacing=spacing)
            
            # Generate the prim paths for all vehicle instances
            target_paths = cloner.generate_paths(self._stage_prefix, self.n_vehicles)

            # Clone the base vehicle to the generated paths
            cloner.clone(source_prim_path=f"{self._stage_prefix}_0", prim_paths=target_paths, replicate_physics=False, copy_from_source=True, base_env_path="/World", root_path=f"{self._stage_prefix}_")

        # Create a view over the root prim of each vehicle
        vehicles = XFormPrimView(prim_paths_expr=f"{self._stage_prefix}_.*/")

        # Retrieve the world poses of the spawned vehicles
        init_pos, init_orientation = vehicles.get_world_poses()

        #init_pos[:, 2] = 0.3
        #vehicles.set_world_poses(init_pos, init_orientation)

        # Store them as tensors for later use
        self.init_pos = torch.as_tensor(init_pos, dtype=torch.float32, device=self.device)
        self.init_orientation = torch.as_tensor(init_orientation, dtype=torch.float32, device=self.device)

        #print(f"Initialized {self.n_vehicles} vehicles at positions: {self.init_pos} and orientations: {self.init_orientation}")


    def _allocate_batch_state(self):
        n = self.n_vehicles
        zeros3 = torch.zeros((n, 3), dtype=torch.float32, device=self.device)
        zeros4 = torch.zeros((n, 4), dtype=torch.float32, device=self.device)

        self._state.position = zeros3.clone()
        self._state.attitude = zeros4.clone()
        self._state.attitude[:, 0] = 1.0
        self._state.linear_velocity = zeros3.clone()
        self._state.linear_body_velocity = zeros3.clone()
        self._state.angular_velocity = zeros3.clone()
        self._state.linear_acceleration = zeros3.clone()

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
        return self._stage_prefix.rpartition("/")[-1]

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
            #for sensor in self._sensors:
            #    sensor.start()

            # Initialize the graphical sensors
            #for graphical_sensor in self._graphical_sensors:
            #    graphical_sensor.start()

            # Intializes the communication with all the backends. This method is invoked automatically when the simulation starts
            for backend in self._backends:
                backend.start()

            # Invoke the start method of the vehicle (if it exists)
            self.start()

        if self._world.is_stopped() and self._sim_running == True:
            self._sim_running = False

            # Stop the sensors
            #for sensor in self._sensors:
            #    sensor.stop()

            # Stop the graphical sensors
            #for graphical_sensor in self._graphical_sensors:
            #    graphical_sensor.stop()

            # Signal all the backends that the simulation has stoped. This method is invoked automatically when the simulation stops
            for backend in self._backends:
                backend.stop()

            self.stop()


    def apply_forces_and_torques_all_parts(self, forces, torques):
        """
        Method that apply forces and torques to all rigid parts matched by the batch view.

        Args:
            forces: Tensor of shape (n_vehicles, n_parts_per_vehicle, 3) or flattened shape (n_total_parts, 3).
            torques: Tensor of shape (n_vehicles, n_parts_per_vehicle, 3) or flattened shape (n_total_parts, 3).

        Notes:
            - Forces/torques are applied in the local body frame when is_global=False.
            - The mapping between tensor rows and prim paths follows self.prims.prim_paths order.
        """

        forces = forces.reshape((self.vehicle_prims.count, 3))
        
        torques = torques.reshape((self.vehicle_prims.count, 3))

        #print("Applying forces:", forces)
        #print("Applying torques:", torques)

        # Apply the force to the rigidbody. The force should be expressed in the rigidbody frame
        self.vehicle_prims.apply_forces_and_torques_at_pos(forces, torques, is_global=False)

    def get_batch_layout_info(self):
        """
        Return structural information about the current batch view.
        """

        return {
            "vehicle_prefix": self._stage_prefix,
            "n_vehicles": self.n_vehicles,
            "n_parts_per_vehicle": self.parts_per_vehicle,
            "n_total_prims": self.vehicle_prims.count,
            "prim_paths": list(self.vehicle_prims.prim_paths),
        }


    def update_state(self, dt: float):
        """
        Method that is called at every physics step to retrieve and update the current state of the vehicle, i.e., get
        the current position, orientation, linear and angular velocities and acceleration of the vehicle.
        The state is defined with respect to the vehicle body prim.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        if self._sim_running == False:
            return

        # Get the positions, orientations, linear velocities, and angular velocities of all vehicle prims in the inertial frame of reference   
        prims_positions, prims_orientations = self.vehicle_prims.get_world_poses()
        prims_linear_vel = self.vehicle_prims.get_linear_velocities()
        prims_angular_vel = self.vehicle_prims.get_angular_velocities()

        # Reshape from (n_vehicles * parts_per_vehicle, 3) to (n_vehicles, parts_per_vehicle, 3)
        prims_positions = torch.as_tensor(prims_positions, dtype=torch.float32, device=self.device).reshape(self.n_vehicles, self.parts_per_vehicle, 3)
        prims_orientations = torch.as_tensor(prims_orientations, dtype=torch.float32, device=self.device).reshape(self.n_vehicles, self.parts_per_vehicle, 4)
        prims_linear_vel = torch.as_tensor(prims_linear_vel, dtype=torch.float32, device=self.device).reshape(self.n_vehicles, self.parts_per_vehicle, 3)
        prims_angular_vel = torch.as_tensor(prims_angular_vel, dtype=torch.float32, device=self.device).reshape(self.n_vehicles, self.parts_per_vehicle, 3)

        # Use the body prim of each vehicle as the reference frame for the state

        # Get the current position of the body in the inertial frame and its orientation relative to the inertial frame
        positions = prims_positions[:, self.body_index, :]
        orientations = prims_orientations[:, self.body_index, :]

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        linear_vel = prims_linear_vel[:, self.body_index, :]
        
        # Get the angular velocity of the vehicle expressed in the body frame of reference
        angular_vel = prims_angular_vel[:, self.body_index, :]

        # Get the linear acceleration of the body relative to the inertial frame, expressed in the inertial frame
        # Note: we must do this approximation, since the Isaac sim does not output the acceleration of the rigid body directly
        linear_acceleration = (linear_vel - self._state.linear_velocity) / dt

        # Update the state
        self._state.position = positions
        self._state.attitude = orientations

        # Express the velocity of the vehicle in the inertial frame X_dot = [x_dot, y_dot, z_dot]
        self._state.linear_velocity = linear_vel

        # The linear velocity V =[u,v,w] of the vehicle's body frame expressed in the body frame of reference
        # Note that: x_dot = Rot * V
        self._state.linear_body_velocity = quaternion_apply(quaternion_invert(self._state.attitude), self._state.linear_velocity)

        # omega = [p,q,r], expressed in the body frame of reference
        self._state.angular_velocity = quaternion_apply(quaternion_invert(self._state.attitude), angular_vel)

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
        every backend. For example, if new data is generated for an IMU and we have a PX4MavlinkBackend, then the update_sensor
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
                    backend._vehicle = self
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
                    backend._vehicle = self
                    backend.update_graphical_sensor(sensor.sensor_type, sensor_data)

    def update_sim_state(self, dt: float):
        """
        Callback that is used to "send" the current state for each backend being used to control the vehicle. This callback
        is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        for backend in self._backends:
            backend._vehicle = self
            backend.update_state(self._state)
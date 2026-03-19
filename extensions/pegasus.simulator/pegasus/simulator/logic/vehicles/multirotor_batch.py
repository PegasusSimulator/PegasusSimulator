"""
| File: multirotor.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Multirotor class which is used as the base for all the multirotor vehicles.
"""
import torch

# The vehicle interface
from pegasus.simulator.logic.vehicles.vehicle_batch import VehicleBatch

# Mavlink interface
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig

# Sensors and dynamics setup
from pegasus.simulator.logic.dynamics import LinearDragBatch
from omni.isaac.core.utils.torch.rotations import quat_rotate_inverse
from pegasus.simulator.logic.thrusters import QuadraticThrustCurveBatch
#from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS

# Extension APIs
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface


class MultirotorBatchConfig:
    """
    A data class that is used for configuring a Multirotor
    """

    def __init__(self, n_vehicles=1):
        """
        Initialization of the MultirotorConfig class
        """
        # Define the same device that is running the simulation
        device = PegasusInterface()._world_settings["device"]

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "quadrotor"

        # The USD file that describes the visual aspect of the vehicle (and some properties such as mass and moments of inertia)
        self.usd_file = ""

        # The default thrust curve for a quadrotor and dynamics relating to drag
        self.thrust_curve = QuadraticThrustCurveBatch(n_vehicles=n_vehicles, device = device)
        self.drag = LinearDragBatch(n_vehicles=n_vehicles, drag_coefficients=[0.50, 0.30, 0.0])

        # The default sensors for a quadrotor
        #self.sensors = [Barometer(device=device), IMU(device=device), Magnetometer(device=device), GPS(device=device)]
        self.sensors = []

        # The default graphical sensors for a quadrotor
        self.graphical_sensors = []

        # The default omnigraphs for a quadrotor
        self.graphs = []

        # The backends for actually sending commands to the vehicle. By default use mavlink (with default mavlink configurations)
        # [Can be None as well, if we do not desired to use PX4 with this simulated vehicle]. It can also be a ROS2 backend
        # or your own custom Backend implementation!
        #self.backends = [PX4MavlinkBackend(config=PX4MavlinkBackendConfig())]
        self.backends = []


class MultirotorBatch(VehicleBatch):
    """Multirotor class - It defines a base interface for creating a multirotor
    """
    def __init__(
        self,
        # Simulation specific configurations
        stage_prefix: str = "quadrotor",
        usd_file: str = "",
        vehicle_batch_id: int = 0,
        n_vehicles: int = 1,
        # Spawning pose of the vehicle
        init_pos=None,
        init_orientation=None,
        spacing: float = 3.0,
        config=None,
    ):
        """Initializes the multirotor object

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. Defaults to "quadrotor".
            usd_file (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            vehicle_batch_id (int): The id to be used for the vehicle batch. Defaults to 0.
            init_pos (list): The initial position of the vehicle in the inertial frame (in ENU convention). Defaults to [0.0, 0.0, 0.07].
            init_orientation (list): The initial orientation of the vehicle in quaternion [qw, qx, qy, qz]. Defaults to [1.0, 0.0, 0.0, 0.0].
            config (MultirotorConfig, optional): Defaults to MultirotorConfig().
        """

        if config is None:
            config = MultirotorBatchConfig(n_vehicles=n_vehicles)

        # 1. Initiate the Vehicle object itself
        super().__init__(stage_prefix, usd_file, n_vehicles, init_pos, init_orientation, config.sensors, config.graphical_sensors, config.graphs, config.backends, spacing)

        # 2. Setup the dynamics of the system - get the thrust curve of the vehicle from the configuration
        self._thrusters = config.thrust_curve
        self._drag = config.drag

        self.input_mode = None
        self._external_forces = None
        self._external_torques = None
        self._desired_rotor_velocities = None

    
    def _cache_rotor_positions_body(self):
        """
        Compute and cache rotor positions relative to the vehicle body frame.

        This only needs to be done once because the rotor locations are fixed
        with respect to the body.
        """

        pos, quat = self.vehicle_prims.get_world_poses()

        # Reshape from (n_vehicles * parts_per_vehicle, 3) to (n_vehicles, parts_per_vehicle, 3)
        pos = torch.as_tensor(pos, dtype=torch.float32, device=self.device).reshape(self.n_vehicles, self.parts_per_vehicle, 3)
        quat = torch.as_tensor(quat, dtype=torch.float32, device=self.device).reshape(self.n_vehicles, self.parts_per_vehicle, 4)
        

        # Extract body and rotor poses
        body_pos = pos[:, self.body_index, :]       # (n_vehicles, 3)
        body_quat = quat[:, self.body_index, :]     # (n_vehicles, 4)
        rotor_pos = pos[:, self.body_index + 1:, :]     # (n_vehicles, num_rotors, 3)

        # Compute rotor positions relative to the body in the world frame
        relative_pos_world = rotor_pos - body_pos.unsqueeze(1)

        # Expand body quaternion so that one quaternion is associated with each rotor
        body_quat_expanded = body_quat.unsqueeze(1).expand(-1, self._thrusters._num_rotors, -1)

        # Rotate relative positions into the body frame
        self._rotor_positions_body = quat_rotate_inverse(body_quat_expanded.reshape(-1, 4), relative_pos_world.reshape(-1, 3)).view(self.n_vehicles, self._thrusters._num_rotors, 3)


    def _cache_allocation_matrix(self):
        """
        Build and cache the control allocation matrix and its pseudo-inverse.

        The matrix maps squared rotor angular velocities to total thrust and body
        torques. Since the vehicle geometry and rotor coefficients are fixed,
        this matrix can be computed once during initialization.
        """

        # Use the first vehicle as reference since all vehicles share the same geometry
        rotor_pos_body = self._rotor_positions_body[0]   # (num_rotors, 3)

        x = rotor_pos_body[:, 0]
        y = rotor_pos_body[:, 1]

        kf = self._thrusters._rotor_constant
        km = self._thrusters._rolling_moment_coefficient
        rot_dir = self._thrusters._rot_dir.to(torch.float32)

        self._allocation_matrix = torch.zeros((4, self._thrusters._num_rotors), dtype=torch.float32, device=self.device)

        # Total thrust contribution
        self._allocation_matrix[0, :] = kf

        # Roll torque contribution: tau_x = y * Fz
        self._allocation_matrix[1, :] = y * kf

        # Pitch torque contribution: tau_y = -x * Fz
        self._allocation_matrix[2, :] = -x * kf

        # Yaw torque contribution from rotor drag
        self._allocation_matrix[3, :] = km * rot_dir

        # Precompute pseudo-inverse for fast control allocation
        self._allocation_inv = torch.linalg.pinv(self._allocation_matrix)

    def start(self):
        """
        Precompute and cache quantities required for multirotor dynamics,
        such as rotor positions relative to the body frame and the control
        allocation matrix. This method is called when the simulation starts.
        """

        # 1. Initialize the vehicle primitives and internal state
        self.initialize()

        # 2. Cache the rotor positions relative to the vehicle body frame.
        #    These are used when computing the forces applied in the simulator.
        self._cache_rotor_positions_body()

        # 3. Cache the control allocation matrix that maps desired body-frame
        #    forces and torques to rotor angular velocities.
        self._cache_allocation_matrix()

    def stop(self):
        """In this case we do not need to do anything extra when the simulation stops"""
        pass

    def update(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in simulation based on the motor speed. 
        This method must be implemented by a class that inherits this type. This callback
        is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        if self._sim_running == False:
            return

        # Call the update methods in all backends
        for backend in self._backends:
            backend._vehicle = self
            backend.update(dt)

        # TODO:  Generate the rotating propeller visual effect
        # self.handle_propeller_visual(i, forces_z[i], articulation)

        # Get the articulation root of the vehicle -> rotating propeller visual effect
        # articulation = ...

        # ------------------------------------------------------------------
        # MODE 1: Direct application of forces and torques
        # ------------------------------------------------------------------
        if self.input_mode == "forces_torques" and self._external_forces is not None and self._external_torques is not None:

            forces = self._external_forces.clone()
            torques = self._external_torques.clone()

            # optional: add drag
            # drag = self._drag.update(self._state, dt)
            # forces[:, 0, :] += drag

        # ------------------------------------------------------------------
        # MODE 2: Rotor angular velocities
        # ------------------------------------------------------------------
        else:
            if self._desired_rotor_velocities is not None:
                desired_rotor_velocities = self._desired_rotor_velocities

            elif len(self._backends) != 0:
                desired_rotor_velocities = self._backends[0].input_reference()

            else:
                desired_rotor_velocities = torch.zeros((self.n_vehicles, self._thrusters._num_rotors), dtype=torch.float32, device=self.device)

            # Input the desired rotor velocities in the thruster model
            self._thrusters.set_input_reference(desired_rotor_velocities)

            forces = torch.zeros((self.n_vehicles, self.parts_per_vehicle, 3), dtype=torch.float32, device=self.device)
            torques = torch.zeros((self.n_vehicles, self.parts_per_vehicle, 3), dtype=torch.float32, device=self.device)

            # Get the desired forces to apply to the rotors vehicles and the desired rolling_moment
            forces_z, _, rolling_moment = self._thrusters.update(self._state, dt)

            # Apply the force in Z to each rotor in the rotor frame
            # Apply the torque to the body frame of the vehicle that corresponds to the rolling moment
            forces[:, 1:, 2] = forces_z
            torques[:, 0, 2] = rolling_moment

            # Compute the total linear drag force to apply to the vehicle's body frame
            drag = self._drag.update(self._state, dt)

            forces[:, 0, :] += drag

        # Apply the forces and torques to the vehicle in the simulator
        self.apply_forces_and_torques_all_parts(forces, torques)


    def force_and_torques_to_velocities(self, force: torch.Tensor, torque: torch.Tensor):
        """
        Auxiliar method used to get the target angular velocities for each rotor, given the total desired thrust [N] and
        torque [Nm] to be applied in the multirotor's body frame.

        Note: This method assumes a quadratic thrust curve. This method will be improved in a future update,
        and a general thrust allocation scheme will be adopted. For now, it is made to work with multirotors directly.

        Args:
            force (torch.Tensor): Desired total thrust along the body Z axis, shape (n_vehicles,)
            torque (torch.Tensor): A vector of the torque to be applied in the body frame of each vehicle [Nm], shape (n_vehicles, 3).

        Returns:
            torch.Tensor: Target rotor angular velocities, shape (n_vehicles, num_rotors).
        """

        # Build desired wrench vector [T, tau_x, tau_y, tau_z]
        wrench = torch.cat((force.unsqueeze(1), torque), dim=1)   # (n_vehicles, 4)

        # Solve for squared angular velocities
        squared_ang_vel = wrench @ self._allocation_inv.T         # (n_vehicles, num_rotors)

        # Clamp negative values caused by the pseudo-inverse
        squared_ang_vel = torch.clamp(squared_ang_vel, min=0.0)

        # Saturate while preserving the relative distribution between rotors
        max_thrust_vel_squared = torch.pow(self._thrusters.max_rotor_velocity[0], 2)
        max_val = torch.max(squared_ang_vel, dim=1, keepdim=True).values

        normalize = torch.clamp(max_val / max_thrust_vel_squared, min=1.0)
        squared_ang_vel = squared_ang_vel / normalize

        # Convert to angular velocity
        ang_vel = torch.sqrt(squared_ang_vel)

        return ang_vel

    
    def set_forces_and_torques(self, forces: torch.Tensor, torques: torch.Tensor):
        assert forces.shape == (self.n_vehicles, self.parts_per_vehicle, 3)
        assert torques.shape == (self.n_vehicles, self.parts_per_vehicle, 3)

        self.input_mode = "forces_torques"
        self._external_forces = forces.to(device=self.device, dtype=torch.float32)
        self._external_torques = torques.to(device=self.device, dtype=torch.float32)


    def set_rotor_velocities(self, rotor_velocities: torch.Tensor):
        assert rotor_velocities.shape == (self.n_vehicles, self._thrusters._num_rotors)

        self.input_mode = "rotor_velocity"
        self._desired_rotor_velocities = rotor_velocities.to(device=self.device, dtype=torch.float32)


    def clear_inputs(self):
        self._external_forces = None
        self._external_torques = None
        self._desired_rotor_velocities = None

# Imports to be able to log to the terminal with fancy colors
import carb

# Imports from the Pegasus library
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Auxiliary scipy and numpy modules
import numpy as np
from scipy.spatial.transform import Rotation

class NonlinearController(Backend):
    """A nonlinear controller class. It implements a nonlinear controller that allows a vehicle to track
    aggressive trajectories. This controlers is well described in the papers
    
    [1] J. Pinto, B. J. Guerreiro and R. Cunha, "Planning Parcel Relay Manoeuvres for Quadrotors," 
    2021 International Conference on Unmanned Aircraft Systems (ICUAS), Athens, Greece, 2021, 
    pp. 137-145, doi: 10.1109/ICUAS51884.2021.9476757.
    [2] D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," 
    2011 IEEE International Conference on Robotics and Automation, Shanghai, China, 2011, 
    pp. 2520-2525, doi: 10.1109/ICRA.2011.5980409.
    """

    def __init__(self, trajectory_file: str, results_file: str=None, id=1):

        # The current rotor references [rad/s]
        self.input_ref = [0.0, 0.0, 0.0, 0.0]

        # The current state of the vehicle expressed in the inertial frame (in ENU)
        self.p = np.zeros((3,))                   # The vehicle position
        self.R: Rotation = Rotation.identity()    # The vehicle attitude
        self.w = np.zeros((3,))                   # The angular velocity of the vehicle
        self.v = np.zeros((3,))                   # The linear velocity of the vehicle in the inertial frame
        self.a = np.zeros((3,))                   # The linear acceleration of the vehicle in the inertial frame

        # Define the control gains matrix for the outer-loop
        self.Kp = np.diag([13.0, 13.0, 13.0])
        self.Kd = np.diag([10.0, 10.0, 10.0])
        self.Kr = np.diag([5.0, 5.0, 5.0])
        self.Kw = np.diag([0.5, 0.5, 0.5])

        # Define the dynamic parameters for the vehicle
        self.m = 1.5        # Mass in Kg
        self.g = 9.81       # The gravity acceleration ms^-2

        # Read the target trajectory from a CSV file inside the trajectories directory
        self.trajectory = self.read_trajectory_from_csv(trajectory_file)
        self.index = 0
        self.max_index, _ = self.trajectory.shape
        self.total_time = 0.0

        # Auxiliar variable, so that we only start sending motor commands once we get the state of the vehicle
        self.reveived_first_state = False

        # Lists used for analysing performance statistics
        self.results_files = results_file
        self.time_vector = []
        self.position_error_over_time = []
        self.velocity_error_over_time = []
        self.atittude_error_over_time = []
        self.attitude_rate_error_over_time = []

        # TODO - remove this - only temporary for vehicle control
        self.id = id

    def read_trajectory_from_csv(self, file_name: str):
        """Auxiliar method used to read the desired trajectory from a CSV file

        Args:
            file_name (str): A string with the name of the trajectory inside the trajectories directory

        Returns:
            np.ndarray: A numpy matrix with the trajectory desired states over time
        """

        # Read the trajectory to a pandas frame
        return np.flip(np.genfromtxt(file_name, delimiter=','), axis=0)


    def start(self):
        """
        Reset the control and trajectory index
        """
        self.index = 0
        self.total_time = 0.0

        # Reset the lists used for analysing performance statistics
        self.time_vector = []
        self.position_error_over_time = []
        self.velocity_error_over_time = []
        self.atittude_error_over_time = []
        self.attitude_rate_error_over_time = []

    def stop(self):
        """
        Stopping the controller. Saving the statistics data for plotting later
        """

        # Check if we should save the statistics to some file or not
        if self.results_files is None:
            return
        
        statistics = {}
        statistics["time"] = np.array(self.time_vector)
        statistics["ep"] = np.vstack(self.position_error_over_time)
        statistics["ev"] = np.vstack(self.velocity_error_over_time)
        statistics["er"] = np.vstack(self.atittude_error_over_time)
        statistics["ew"] = np.vstack(self.attitude_rate_error_over_time)
        np.savez(self.results_files, **statistics)
        carb.log_warn("Statistics saved to: " + self.results_files)

    def update_sensor(self, sensor_type: str, data):
        """
        Do nothing. For now ignore all the sensor data and just use the state directly for demonstration purposes. 
        This is a callback that is called at every physics step.

        Args:
            sensor_type (str): The name of the sensor providing the data
            data (dict): A dictionary that contains the data produced by the sensor
        """
        pass

    def update_state(self, state: State):
        """
        Method that updates the current state of the vehicle. This is a callback that is called at every physics step

        Args:
            state (State): The current state of the vehicle.
        """
        self.p = state.position
        self.R = Rotation.from_quat(state.attitude)
        self.w = state.angular_velocity
        self.v = state.linear_velocity

        self.reveived_first_state = True

    def input_reference(self):
        """
        Method that is used to return the latest target angular velocities to be applied to the vehicle

        Returns:
            A list with the target angular velocities for each individual rotor of the vehicle
        """
        return self.input_ref

    def update(self, dt: float):
        """Method that implements the nonlinear control law and updates the target angular velocities for each rotor. 
        This method will be called by the simulation on every physics step

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        
        if self.reveived_first_state == False:
            return

        # -------------------------------------------------
        # Update the references for the controller to track
        # -------------------------------------------------
        self.total_time += dt
        

        # Check if we need to update to the next trajectory index
        if self.index < self.max_index - 1 and self.total_time >= self.trajectory[self.index + 1, 0]:
            self.index += 1

        # the target positions [m], velocity [m/s], accelerations [m/s^2], jerk [m/s^3], yaw-angle [rad], yaw-rate [rad/s]
        p_ref = np.array([self.trajectory[self.index, 1], self.trajectory[self.index, 2], self.trajectory[self.index, 3]])
        v_ref = np.array([self.trajectory[self.index, 4], self.trajectory[self.index, 5], self.trajectory[self.index, 6]])
        a_ref = np.array([self.trajectory[self.index, 7], self.trajectory[self.index, 8], self.trajectory[self.index, 9]])
        j_ref = np.array([self.trajectory[self.index, 10], self.trajectory[self.index, 11], self.trajectory[self.index, 12]])
        yaw_ref = self.trajectory[self.index, 13]
        yaw_rate_ref = self.trajectory[self.index, 14]

        # -------------------------------------------------
        # Start the controller implementation
        # -------------------------------------------------

        # Compute the tracking errors
        ep = self.p - p_ref
        ev = self.v - v_ref

        # Compute F_des term
        F_des = -(self.Kp @ ep) - (self.Kd @ ev) + np.array([0.0, 0.0, self.m * self.g]) + (self.m * a_ref)

        # Get the current axis Z_B (given by the last column of the rotation matrix)
        Z_B = self.R.as_matrix()[:,2]

        # Get the desired total thrust in Z_B direction (u_1)
        u_1 = F_des @ Z_B

        # Compute the desired body-frame axis Z_b
        Z_b_des = F_des / np.linalg.norm(F_des)

        # Compute X_C_des 
        X_c_des = np.array([np.cos(yaw_ref), np.sin(yaw_ref), 0.0])

        # Compute Y_b_des
        Z_b_cross_X_c = np.cross(Z_b_des, X_c_des)
        Y_b_des = Z_b_cross_X_c / np.linalg.norm(Z_b_cross_X_c)

        # Compute X_b_des
        X_b_des = np.cross(Y_b_des, Z_b_des)

        # Compute the desired rotation R_des = [X_b_des | Y_b_des | Z_b_des]
        R_des = np.c_[X_b_des, Y_b_des, Z_b_des]
        R = self.R.as_matrix()

        # Compute the rotation error
        e_R = 0.5 * self.vee((R_des.T @ R) - (R.T @ R_des))

        # Compute an approximation of the current vehicle acceleration in the inertial frame (since we cannot measure it directly)
        self.a = (u_1 * Z_B) / self.m - np.array([0.0, 0.0, self.g])

        # Compute the derivative of the acceleration. Since we cannot measure this directly, but we know exactly
        # the acceleration input that we are giving the vehicle, i.e. u_1 = F_des @ Z_B, assuming there is no
        # saturation and no time delays (ideal system), then u_1_dot \approx F_des_dot @ Z_B
        # Note: This is left here, because I still believe we can use the error, instead of feeding-forward 
        # only the jerk for the projection, but I need to think a little bit more on the subject.
        # For now, this is already pretty good 
        #ea = self.a - a_ref
        #F_des_dot = -(self.Kp @ ev) - (self.Kd @ ea) + (self.m * j_ref)
        #u_1_dot = F_des_dot @ Z_B

        # Compute the desired angular velocity by projecting the angular velocity in the Xb-Yb plane
        #projection of angular velocity on xB − yB plane
        # see eqn (7) from [2].
        hw = (self.m / u_1) * (j_ref - np.dot(Z_b_des, j_ref) * Z_b_des) 
        
        # desired angular velocity
        w_des = np.array([-np.dot(hw, Y_b_des), 
                           np.dot(hw, X_b_des), 
                           yaw_rate_ref * Z_b_des[2]])

        # Compute the angular velocity error
        e_w = self.w - w_des

        # Compute the torques to apply on the rigid body
        tau = -(self.Kr @ e_R) - (self.Kw @ e_w)

        # ----------------------------
        # Statistics to save for later
        # ----------------------------
        self.time_vector.append(self.total_time)        
        self.position_error_over_time.append(ep)
        self.velocity_error_over_time.append(ev)
        self.atittude_error_over_time.append(e_R)
        self.attitude_rate_error_over_time.append(e_w)

        # TODO - replace these 3 lines by proper force allocation matrix
        vehicle = PegasusInterface().vehicle_manager.get_vehicle("/World/quadrotor" + str(self.id))
        #vehicle.apply_torque(tau)
        #vehicle.apply_force(np.array([0.0, 0.0, u_1]))

        # Apply the force and torque directly and let the multirotor apply
        # the forces to each rotor individually via its built in API
        self.input_ref = self.apply_force_and_torques(u_1, tau)
        
    
    def apply_force_and_torques(self, force: float, torque: np.ndarray):
        """Method that given the total force [N] and the torque vector [\tau_x, \tau_y, \tau_z]^T [Nm]
        computes the actual angular velocities to apply to each rotor of the vehicle.

        Args:
            force (float): The total force [N] to apply to the vehicle body frame
            torque (np.ndarray): The total torque vector [\tau_x, \tau_y, \tau_z]^T [Nm] to apply to the vehicle
        """

        # Get the vehicle
        vehicle = PegasusInterface().vehicle_manager.get_vehicle("/World/quadrotor" + str(self.id))
        return vehicle.force_and_torques_to_velocities(force, torque)

    @staticmethod
    def vee(S):
        """Auxiliary function that computes the 'v' map which takes elements from so(3) to R^3.

        Args:
            S (np.array): A matrix in so(3)
        """
        return np.array([-S[1,2], S[0,2], -S[0,1]])
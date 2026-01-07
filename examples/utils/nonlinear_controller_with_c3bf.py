#!/usr/bin/env python
"""
| File: nonlinear_controller.py (Baseline, no CBF)
| Author: Marcelo Jacinto and Joao Pinto (original), modified by Shakthi for baseline comparison
| License: BSD-3-Clause.
"""

import carb

# Pegasus
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends import Backend

# SciPy / NumPy
import numpy as np
from scipy.spatial.transform import Rotation

# For plots
import matplotlib.pyplot as plt
import os


class NonlinearController(Backend):
    """Nonlinear quadrotor controller (baseline, no CBF)."""

    def __init__(
        self,
        trajectory_file: str = None,
        results_file: str = None,
        reverse=False,
        Kp=[10.0, 10.0, 10.0],
        Kd=[8.5, 8.5, 8.5],
        Ki=[1.50, 1.50, 1.50],
        Kr=[3.5, 3.5, 3.5],
        Kw=[0.5, 0.5, 0.5],
    ):

        # Rotor angular velocity references [rad/s]
        self.input_ref = [0.0, 0.0, 0.0, 0.0]

        # Vehicle state (ENU)
        self.p = np.zeros((3,))
        self.R: Rotation = Rotation.identity()
        self.w = np.zeros((3,))
        self.v = np.zeros((3,))
        self.a = np.zeros((3,))

        # Outer-loop gains
        self.Kp = np.diag(Kp)
        self.Kd = np.diag(Kd)
        self.Ki = np.diag(Ki)
        self.Kr = np.diag(Kr)
        self.Kw = np.diag(Kw)

        self.int = np.array([0.0, 0.0, 0.0])

        # Dynamics params
        self.m = 1.50
        self.g = 9.81

        # Trajectory handling
        self.index = 0
        if trajectory_file is not None:
            self.trajectory = self.read_trajectory_from_csv(trajectory_file)
            self.max_index, _ = self.trajectory.shape
            self.total_time = 0.0
        else:
            self.total_time = -5.0
            self.trajectory = None
            self.max_index = 1

        self.reverse = reverse
        self.reveived_first_state = False

        # Stats / logs
        self.results_files = results_file
        self.time_vector = []
        self.desired_position_over_time = []
        self.position_over_time = []
        self.position_error_over_time = []
        self.velocity_error_over_time = []
        self.atittude_error_over_time = []
        self.attitude_rate_error_over_time = []

        # Waypoint-based navigation (same as CBF case)
        self.waypoints = [
            np.array([0.0, 0.0, 1.5]),  # start / takeoff
            np.array([6.0, 0.0, 1.5]),  # forward
        ]
        self.current_wp_idx = 0
        self.wp_reached_radius = 0.2
        self.k_wp = 0.8
        self.v_nominal_max = 1.5

        # Integrated nominal reference path (for plotting)
        self.ref_initialized = False
        self.p_ref_nom = None

        # Extra logs for plots
        self.v_nominal_over_time = []   # 3D nominal velocity
        self.min_dist_over_time = []    # min distance to any obstacle
        self.rpy_over_time = []         # roll/pitch/yaw [deg]

    # --------------------------------------------------------------------------
    # Pegasus interface methods
    # --------------------------------------------------------------------------
    def read_trajectory_from_csv(self, file_name: str):
        return np.flip(np.genfromtxt(file_name, delimiter=","), axis=0)

    def start(self):
        self.reset_statistics()

    def stop(self):
        """
        Save statistics and generate baseline plots.
        """

        if len(self.time_vector) == 0:
            carb.log_warn("No data collected in baseline controller.")
            return

        # Stack core logs
        t = np.array(self.time_vector)
        p = np.vstack(self.position_over_time)
        p_ref = np.vstack(self.desired_position_over_time)
        ep = np.vstack(self.position_error_over_time)
        ev = np.vstack(self.velocity_error_over_time)
        eR = np.vstack(self.atittude_error_over_time)
        ew = np.vstack(self.attitude_rate_error_over_time)

        v_nominal = (
            np.vstack(self.v_nominal_over_time)
            if len(self.v_nominal_over_time) > 0
            else None
        )
        min_dist = (
            np.array(self.min_dist_over_time)
            if len(self.min_dist_over_time) > 0
            else None
        )
        rpy = (
            np.vstack(self.rpy_over_time)
            if len(self.rpy_over_time) > 0
            else None
        )

        # Save basic statistics
        if self.results_files is not None:
            statistics = {
                "time": t,
                "p": p,
                "desired_p": p_ref,
                "ep": ep,
                "ev": ev,
                "er": eR,
                "ew": ew,
            }
            np.savez(self.results_files, **statistics)
            carb.log_warn(f"[BASELINE] Statistics saved to: {self.results_files}")

        # 1) Tracking metrics
        pos_err = p - p_ref
        rmse = np.sqrt(np.mean(pos_err ** 2, axis=0))
        mae = np.mean(np.linalg.norm(pos_err, axis=1))
        print("\n====== Baseline Tracking Metrics ======")
        print(f"RMSE_x: {rmse[0]:.4f} m")
        print(f"RMSE_y: {rmse[1]:.4f} m")
        print(f"RMSE_z: {rmse[2]:.4f} m")
        print(f"MAE_pos_norm: {mae:.4f} m")
        print("======================================\n")

        # 2) Safety metrics (shows collision)
        if min_dist is not None and len(min_dist) > 0:
            min_overall = np.min(min_dist)
            drone_radius = 0.2
            r_obs_max = 0.5
            safe_radius = drone_radius + r_obs_max
            collision_occurred = np.any(min_dist < 0.0)
            print("====== Baseline Safety Metrics ======")
            print(f"Minimum distance to any obstacle: {min_overall:.4f} m")
            print(f"Effective safe radius (drone + max obstacle): {safe_radius:.4f} m")
            print(
                f"Collision occurred? {'YES (as expected baseline has no safety filter)' if collision_occurred else 'NO'}"
            )
            print("=====================================\n")

        # ----------------- PLOTS -----------------
        fig_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "figures_baseline")
        os.makedirs(fig_dir, exist_ok=True)

        # (1) 3D Trajectory
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111, projection="3d")
        ax1.plot(p[:, 0], p[:, 1], p[:, 2], label="Actual", linewidth=2)
        ax1.plot(p_ref[:, 0], p_ref[:, 1], p_ref[:, 2], "--", label="Nominal Ref", linewidth=2)
        ax1.set_xlabel("X [m]")
        ax1.set_ylabel("Y [m]")
        ax1.set_zlabel("Z [m]")
        ax1.set_title("3D Trajectory (Baseline, no CBF)")
        ax1.legend()
        ax1.grid(True)
        fig1.savefig(os.path.join(fig_dir, "traj_3d_baseline.png"), dpi=300, bbox_inches="tight")

        # (2) Top-down XY with obstacles
        fig2 = plt.figure()
        ax2 = fig2.add_subplot(111)
        ax2.plot(p[:, 0], p[:, 1], label="Actual", linewidth=2)
        ax2.plot(p_ref[:, 0], p_ref[:, 1], "--", label="Nominal Ref", linewidth=2)

        # same obstacles as CBF case
        obstacles_xy = [
            (np.array([-1.0, -1.7]), 0.5 + 0.2),
            (np.array([-2.3, 0.5]), 0.5+0.2),
        ]
        for center, rad in obstacles_xy:
            circle = plt.Circle(center, rad, fill=False, linestyle="--")
            ax2.add_patch(circle)
            ax2.plot(center[0], center[1], "rx")

        ax2.set_xlabel("X [m]")
        ax2.set_ylabel("Y [m]")
        ax2.set_title("Top-Down View: Baseline (No CBF)")
        ax2.axis("equal")
        ax2.legend()
        ax2.grid(True)
        fig2.savefig(os.path.join(fig_dir, "topdown_baseline.png"), dpi=300, bbox_inches="tight")

        # (3) Tracking errors
        fig3 = plt.figure()
        ax3 = fig3.add_subplot(111)
        ax3.plot(t, pos_err[:, 0], label="e_x")
        ax3.plot(t, pos_err[:, 1], label="e_y")
        ax3.plot(t, pos_err[:, 2], label="e_z")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Position Error [m]")
        ax3.set_title("Tracking Errors (Baseline)")
        ax3.legend()
        ax3.grid(True)
        fig3.savefig(os.path.join(fig_dir, "tracking_errors_baseline.png"), dpi=300, bbox_inches="tight")

        # (4) Velocity profile (nominal only)
        if v_nominal is not None:
            fig4 = plt.figure()
            ax4 = fig4.add_subplot(111)
            ax4.plot(t, np.linalg.norm(v_nominal, axis=1), label="||v_nominal||")
            ax4.set_xlabel("Time [s]")
            ax4.set_ylabel("Speed [m/s]")
            ax4.set_title("Velocity Profile (Baseline Nominal Speed)")
            ax4.legend()
            ax4.grid(True)
            fig4.savefig(os.path.join(fig_dir, "velocity_profile_baseline.png"), dpi=300, bbox_inches="tight")

        # (5) Distance to obstacle over time
        if min_dist is not None:
            fig5 = plt.figure()
            ax5 = fig5.add_subplot(111)
            ax5.plot(t, min_dist, label="Min distance to any obstacle")
            ax5.axhline(0.0, color="r", linestyle="--", label="Collision boundary")
            ax5.set_xlabel("Time [s]")
            ax5.set_ylabel("Distance [m]")
            ax5.set_title("Distance to Closest Obstacle (Baseline)")
            ax5.legend()
            ax5.grid(True)
            fig5.savefig(os.path.join(fig_dir, "distance_to_obstacle_baseline.png"), dpi=300, bbox_inches="tight")

        # (6) Attitude (roll/pitch/yaw)
        if rpy is not None:
            fig6 = plt.figure()
            ax6 = fig6.add_subplot(111)
            ax6.plot(t, rpy[:, 0], label="roll [deg]")
            ax6.plot(t, rpy[:, 1], label="pitch [deg]")
            ax6.plot(t, rpy[:, 2], label="yaw [deg]")
            ax6.set_xlabel("Time [s]")
            ax6.set_ylabel("Angle [deg]")
            ax6.set_title("Attitude Over Time (Baseline)")
            ax6.legend()
            ax6.grid(True)
            fig6.savefig(os.path.join(fig_dir, "attitude_rpy_baseline.png"), dpi=300, bbox_inches="tight")

        carb.log_warn(f"[BASELINE] Saved baseline figures to: {fig_dir}")

        self.reset_statistics()

    def update_sensor(self, sensor_type: str, data):
        pass

    def update_state(self, state: State):
        self.p = state.position
        self.R = Rotation.from_quat(state.attitude)
        self.w = state.angular_velocity
        self.v = state.linear_velocity
        self.reveived_first_state = True

    def input_reference(self):
        return self.input_ref

    def update(self, dt: float):
        if not self.reveived_first_state:
            return

        self.total_time += dt

        # -------------------------------------------------
        # Reference generation
        # -------------------------------------------------
        if self.trajectory is not None:
            # CSV trajectory mode (unchanged)
            if self.index < self.max_index - 1 and self.total_time >= self.trajectory[self.index + 1, 0]:
                self.index += 1

            p_ref = np.array(
                [self.trajectory[self.index, 1],
                 self.trajectory[self.index, 2],
                 self.trajectory[self.index, 3]]
            )
            v_ref = np.array(
                [self.trajectory[self.index, 4],
                 self.trajectory[self.index, 5],
                 self.trajectory[self.index, 6]]
            )
            a_ref = np.array(
                [self.trajectory[self.index, 7],
                 self.trajectory[self.index, 8],
                 self.trajectory[self.index, 9]]
            )
            j_ref = np.array(
                [self.trajectory[self.index, 10],
                 self.trajectory[self.index, 11],
                 self.trajectory[self.index, 12]]
            )
            yaw_ref = self.trajectory[self.index, 13]
            yaw_rate_ref = self.trajectory[self.index, 14]

        else:
            # Waypoint-based point-to-point (same as CBF case but no safety filter)
            goal = self.waypoints[self.current_wp_idx]
            pos_err_wp = goal - self.p
            dist_wp = np.linalg.norm(pos_err_wp)

            if dist_wp < self.wp_reached_radius and self.current_wp_idx < len(self.waypoints) - 1:
                self.current_wp_idx += 1
                goal = self.waypoints[self.current_wp_idx]
                pos_err_wp = goal - self.p
                dist_wp = np.linalg.norm(pos_err_wp)

            # Nominal velocity toward waypoint
            if dist_wp > 1e-3:
                v_nominal = self.k_wp * pos_err_wp
                v_norm = np.linalg.norm(v_nominal)
                if v_norm > self.v_nominal_max:
                    v_nominal = v_nominal * (self.v_nominal_max / v_norm)
            else:
                v_nominal = np.zeros(3)

            # Integrated nominal reference path
            if not self.ref_initialized:
                self.p_ref_nom = self.p.copy()
                self.ref_initialized = True
            self.p_ref_nom = self.p_ref_nom + v_nominal * dt

            p_ref = self.p_ref_nom.copy()
            v_ref = v_nominal.copy()
            a_ref = np.zeros(3)
            j_ref = np.zeros(3)
            yaw_ref = 0.0
            yaw_rate_ref = 0.0

        # -------------------------------------------------
        # Baseline controller (same as original, no CBF)
        # -------------------------------------------------
        ep = self.p - p_ref
        ev = self.v - v_ref
        self.int = self.int + ep * dt
        ei = self.int

        F_des = (
            -(self.Kp @ ep)
            - (self.Kd @ ev)
            - (self.Ki @ ei)
            + np.array([0.0, 0.0, self.m * self.g])
            + (self.m * a_ref)
        )

        Z_B = self.R.as_matrix()[:, 2]
        u_1 = F_des @ Z_B

        Z_b_des = F_des / np.linalg.norm(F_des)

        X_c_des = np.array([np.cos(yaw_ref), np.sin(yaw_ref), 0.0])
        Z_b_cross_X_c = np.cross(Z_b_des, X_c_des)
        Y_b_des = Z_b_cross_X_c / np.linalg.norm(Z_b_cross_X_c)
        X_b_des = np.cross(Y_b_des, Z_b_des)

        R_des = np.c_[X_b_des, Y_b_des, Z_b_des]
        R = self.R.as_matrix()
        e_R = 0.5 * self.vee((R_des.T @ R) - (R.T @ R_des))

        self.a = (u_1 * Z_B) / self.m - np.array([0.0, 0.0, self.g])

        hw = (self.m / u_1) * (j_ref - np.dot(Z_b_des, j_ref) * Z_b_des)
        w_des = np.array(
            [-np.dot(hw, Y_b_des), np.dot(hw, X_b_des), yaw_rate_ref * Z_b_des[2]]
        )

        e_w = self.w - w_des
        tau = -(self.Kr @ e_R) - (self.Kw @ e_w)

        if self.vehicle:
            self.input_ref = self.vehicle.force_and_torques_to_velocities(u_1, tau)

        # ----------------------------
        # Extra logging for plots
        # ----------------------------
        self.time_vector.append(self.total_time)
        self.position_over_time.append(self.p.copy())
        self.desired_position_over_time.append(p_ref.copy())
        self.position_error_over_time.append(ep.copy())
        self.velocity_error_over_time.append(ev.copy())
        self.atittude_error_over_time.append(e_R.copy())
        self.attitude_rate_error_over_time.append(e_w.copy())

        # Log nominal velocity
        self.v_nominal_over_time.append(v_ref.copy())

        # Obstacles (same as CBF experiment) – only for distance logging
        obstacles = [
            {
                "shape": "cylinder_z",                     
                "c": np.array([-1.0, -1.4, 1.2]),          
                "z_min": 0.0,                              
                "z_max": 2.0,                              
                "c_dot": np.zeros(3),
                "r_obs": 0.5                               
            },

            {
                "shape": "cylinder_z",
                "c": np.array([-2.3, 0.5, 1.2]),
                "z_min": 0.0,
                "z_max": 2.0,
                "c_dot": np.zeros(3),
                "r_obs": 0.5
            }
        ]
        drone_radius = 0.2
        min_dist = np.inf
        for obs in obstacles:
            center = obs["c"]
            r_obs = obs["r_obs"]
            shape = obs.get("shape", "sphere")

            if shape == "sphere":
                d_raw = np.linalg.norm(self.p - center) - (drone_radius + r_obs)

            elif shape == "cylinder_z":
                z_min = obs.get("z_min", -np.inf)
                z_max = obs.get("z_max",  np.inf)

                # If outside vertical extent, treat as "far"
                if self.p[2] < (z_min - drone_radius) or self.p[2] > (z_max + drone_radius):
                    d_raw = np.inf
                else:
                    delta_xy = self.p[:2] - center[:2]
                    d_raw = np.linalg.norm(delta_xy) - (drone_radius + r_obs)

            else:
                # unknown shape → ignore
                d_raw = np.inf

            if d_raw < min_dist:
                min_dist = d_raw

        self.min_dist_over_time.append(min_dist)

        # Attitude log
        rpy_deg = self.R.as_euler("xyz", degrees=True)
        self.rpy_over_time.append(rpy_deg)

    # ----------------------------------------------------------------------
    @staticmethod
    def vee(S):
        return np.array([-S[1, 2], S[0, 2], -S[0, 1]])

    def reset_statistics(self):
        self.index = 0
        if self.trajectory is not None:
            self.total_time = 0.0
        else:
            self.total_time = -5.0

        self.time_vector = []
        self.desired_position_over_time = []
        self.position_over_time = []
        self.position_error_over_time = []
        self.velocity_error_over_time = []
        self.atittude_error_over_time = []
        self.attitude_rate_error_over_time = []

        # waypoint / ref logs
        self.ref_initialized = False
        self.p_ref_nom = None
        self.v_nominal_over_time = []
        self.min_dist_over_time = []
        self.rpy_over_time = []

    # ----------------------------------------------------------------------
    # Built-in exponential trajectory (unused in waypoint mode, kept for compatibility)
    # ----------------------------------------------------------------------
    def pd(self, t, s, reverse=False):
        x = t
        z = 1 / s * np.exp(-0.5 * np.power(t / s, 2)) + 1.0
        y = 1 / s * np.exp(-0.5 * np.power(t / s, 2))
        if reverse:
            y = -1 / s * np.exp(-0.5 * np.power(t / s, 2)) + 4.5
        return np.array([x, y, z])

    def d_pd(self, t, s, reverse=False):
        x = 1.0
        y = -(t * np.exp(-np.power(t, 2) / (2 * np.power(s, 2)))) / np.power(s, 3)
        z = y
        if reverse:
            y = (t * np.exp(-np.power(t, 2) / (2 * np.power(s, 2)))) / np.power(s, 3)
        return np.array([x, y, z])

    def dd_pd(self, t, s, reverse=False):
        x = 0.0
        y = (np.power(t, 2) * np.exp(-np.power(t, 2) / (2 * np.power(s, 2)))) / np.power(
            s, 5
        ) - np.exp(-np.power(t, 2) / (2 * np.power(s, 2))) / np.power(s, 3)
        z = y
        if reverse:
            y = np.exp(-np.power(t, 2) / (2 * np.power(s, 2))) / np.power(
                s, 3
            ) - (np.power(t, 2) * np.exp(-np.power(t, 2) / (2 * np.power(s, 2)))) / np.power(
                s, 5
            )
        return np.array([x, y, z])

    def ddd_pd(self, t, s, reverse=False):
        x = 0.0
        y = (3 * t * np.exp(-np.power(t, 2) / (2 * np.power(s, 2)))) / np.power(
            s, 5
        ) - (np.power(t, 3) * np.exp(-np.power(t, 2) / (2 * np.power(s, 2)))) / np.power(
            s, 7
        )
        z = y
        if reverse:
            y = (np.power(t, 3) * np.exp(-np.power(t, 2) / (2 * np.power(s, 2)))) / np.power(
                s, 7
            ) - (3 * t * np.exp(-np.power(t, 2) / (2 * np.power(s, 2)))) / np.power(
                s, 5
            )
        return np.array([x, y, z])

    def yaw_d(self, t, s):
        return 0.0

    def d_yaw_d(self, t, s):
        return 0.0

    def reset(self):
        pass

    def update_graphical_sensor(self, sensor_type: str, data):
        pass

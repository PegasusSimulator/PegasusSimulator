"""
| File: gps.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Simulates a gps. Based on the implementation provided in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo) by Amy Wagoner and Nuno Marques
"""
__all__ = ["GPS"]

import torch

from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.sensors.geo_mag_utils import reprojection

class GPS(Sensor):
    """The class that implements a GPS sensor. This class inherits the base class Sensor.
    """

    def __init__(self, config={}, device="cpu"):
        """Initialize the GPS class.

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the GPS - it can be empty or only have some of the parameters used by the GPS.
        
        Examples:
            The dictionary default parameters are

            >>> {"fix_type": 3,
            >>>  "eph": 1.0,
            >>>  "epv": 1.0,
            >>>  "sattelites_visible": 10,
            >>>  "gps_xy_random_walk": 2.0,         # (m/s) / sqrt(hz)
            >>>  "gps_z_random_walk": 4.0,          # (m/s) / sqrt(hz)
            >>>  "gps_xy_noise_density": 2.0e-4,    # (m) / sqrt(hz)
            >>>  "gps_z_noise_density": 4.0e-4,     # (m) / sqrt(hz)
            >>>  "gps_vxy_noise_density": 0.2,      # (m/s) / sqrt(hz)
            >>>  "gps_vz_noise_density": 0.4,       # (m/s) / sqrt(hz)
            >>>  "gps_correlation_time": 60,        # s
            >>>  "update_rate": 1.0                 # Hz
            >>> }
        """

        # Set device
        self.device = device

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="GPS", update_rate=torch.tensor(config.get("update_rate", 250.0), dtype=torch.float32, device=device))

        # Define the GPS simulated/fixed values
        self._fix_type = torch.tensor(config.get("fix_type", 3), dtype=torch.int32, device=device)
        self._eph = torch.tensor(config.get("eph", 1.0), dtype=torch.float32, device=device)
        self._epv = torch.tensor(config.get("epv", 1.0), dtype=torch.float32, device=device)
        self._sattelites_visible = torch.tensor(config.get("sattelites_visible", 10), dtype=torch.int32, device=device)

        # Parameters for GPS random walk
        self._random_walk_gps = torch.zeros(3, dtype=torch.float32, device=device)
        self._gps_xy_random_walk = torch.tensor(config.get("gps_xy_random_walk", 2.0), dtype=torch.float32, device=device)  # (m/s) / sqrt(hz)
        self._gps_z_random_walk = torch.tensor(config.get("gps_z_random_walk", 4.0), dtype=torch.float32, device=device)  # (m/s) / sqrt(hz)

        # Parameters for the position noise
        self._noise_gps_pos = torch.zeros(3, dtype=torch.float32, device=device)
        self._gps_xy_noise_density = torch.tensor(config.get("gps_xy_noise_density", 2.0e-4), dtype=torch.float32, device=device)  # (m) / sqrt(hz)
        self._gps_z_noise_density = torch.tensor(config.get("gps_z_noise_density", 4.0e-4), dtype=torch.float32, device=device)  # (m) / sqrt(hz)

        # Parameters for the velocity noise
        self._noise_gps_vel = torch.zeros(3, dtype=torch.float32, device=device)
        self._gps_vxy_noise_density = torch.tensor(config.get("gps_vxy_noise_density", 0.2), dtype=torch.float32, device=device)  # (m/s) / sqrt(hz)
        self._gps_vz_noise_density = torch.tensor(config.get("gps_vz_noise_density", 0.4), dtype=torch.float32, device=device)  # (m/s) / sqrt(hz)

        # Parameters for the GPS bias
        self._gps_bias = torch.zeros(3, dtype=torch.float32, device=device)
        self._gps_correlation_time = torch.tensor(config.get("gps_correlation_time", 60), dtype=torch.float32, device=device)

        # Save the current state measured by the GPS (and initialize at the origin)
        self._state = {
            "latitude": torch.deg2rad(self._origin_lat),
            "longitude": torch.deg2rad(self._origin_lon),
            "altitude": self._origin_alt,
            "speed": torch.tensor(0.0, dtype=torch.float32, device=device),
            "velocity_north": torch.tensor(0.0, dtype=torch.float32, device=device),
            "velocity_east": torch.tensor(0.0, dtype=torch.float32, device=device),
            "velocity_down": torch.tensor(0.0, dtype=torch.float32, device=device),
            # Constant values
            "fix_type": self._fix_type,
            "eph": self._eph,
            "epv": self._epv,
            "cog": torch.tensor(0.0, dtype=torch.float32, device=device),
            "sattelites_visible": self._sattelites_visible,
            "latitude_gt": torch.deg2rad(self._origin_lat),
            "longitude_gt": torch.deg2rad(self._origin_lon),
            "altitude_gt": self._origin_alt,
        }

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state

    @Sensor.update_at_rate
    def update(self, state: torch.Tensor, dt: torch.Tensor):
        """Method that implements the logic of a gps. In this method we start by generating the GPS bias terms which are then
        added to the real position of the vehicle, expressed in ENU inertial frame. This position affected by noise
        is reprojected in order to obtain the corresponding latitude and longitude. Additionally, to the linear velocity, noise
        is added.

        Args:
            state (State): The current state of the vehicle.
            dt (torch.Tensor): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        # Update noise parameters
        self._random_walk_gps[0] = self._gps_xy_random_walk * torch.sqrt(dt) * torch.randn((), device=self.device)
        self._random_walk_gps[1] = self._gps_xy_random_walk * torch.sqrt(dt) * torch.randn((), device=self.device)
        self._random_walk_gps[2] = self._gps_z_random_walk * torch.sqrt(dt) * torch.randn((), device=self.device)

        self._noise_gps_pos[0] = self._gps_xy_noise_density * torch.sqrt(dt) * torch.randn((), device=self.device)
        self._noise_gps_pos[1] = self._gps_xy_noise_density * torch.sqrt(dt) * torch.randn((), device=self.device)
        self._noise_gps_pos[2] = self._gps_z_noise_density * torch.sqrt(dt) * torch.randn((), device=self.device)

        self._noise_gps_vel[0] = self._gps_vxy_noise_density * torch.sqrt(dt) * torch.randn((), device=self.device)
        self._noise_gps_vel[1] = self._gps_vxy_noise_density * torch.sqrt(dt) * torch.randn((), device=self.device)
        self._noise_gps_vel[2] = self._gps_vz_noise_density * torch.sqrt(dt) * torch.randn((), device=self.device)

        # Perform GPS bias integration (using euler integration -> to be improved)
        self._gps_bias[0] = (
            self._gps_bias[0] + self._random_walk_gps[0] * dt - self._gps_bias[0] / self._gps_correlation_time
        )
        self._gps_bias[1] = (
            self._gps_bias[1] + self._random_walk_gps[1] * dt - self._gps_bias[1] / self._gps_correlation_time
        )
        self._gps_bias[2] = (
            self._gps_bias[2] + self._random_walk_gps[2] * dt - self._gps_bias[2] / self._gps_correlation_time
        )

        # reproject position with noise into geographic coordinates
        pos_with_noise: torch.Tensor = state.position + self._noise_gps_pos + self._gps_bias
        latitude, longitude = reprojection(pos_with_noise, torch.deg2rad(self._origin_lat), torch.deg2rad(self._origin_lon))

        # Compute the values of the latitude and longitude without noise (for groundtruth measurements)
        latitude_gt, longitude_gt = reprojection(
            state.position, torch.deg2rad(self._origin_lat), torch.deg2rad(self._origin_lon)
        )

        # Add noise to the velocity expressed in the world frame
        velocity: torch.Tensor = state.linear_velocity  # + self._noise_gps_vel

        # Compute the xy speed
        speed: torch.Tensor = torch.linalg.norm(velocity[:2])

        # Course over ground (NOT heading, but direction of movement),
        # 0.0..359.99 degrees. If unknown, set to: 65535 [cdeg] (type:uint16_t)
        ve = velocity[0]
        vn = velocity[1]
        cog = torch.rad2deg(torch.arctan2(ve, vn))

        if (cog < 0.0).item():
            cog = cog + 360.0

        cog = cog * 100

        # Add the values to the dictionary and return it
        self._state = {
            "latitude": torch.rad2deg(latitude),
            "longitude": torch.rad2deg(longitude),
            "altitude": state.position[2] + self._origin_alt - self._noise_gps_pos[2] + self._gps_bias[2],
            "speed": speed,
            # Conversion from ENU (standard of Isaac Sim to NED - used in GPS sensors)
            "velocity_north": velocity[1],
            "velocity_east": velocity[0],
            "velocity_down": -velocity[2],
            # Constant values
            "fix_type": self._fix_type,
            "eph": self._eph,
            "epv": self._epv,
            "cog": torch.tensor(0.0, dtype=torch.float32, device=self.device),  # cog,
            "sattelites_visible": self._sattelites_visible,
            "latitude_gt": latitude_gt,
            "longitude_gt": longitude_gt,
            "altitude_gt": state.position[2] + self._origin_alt,
        }

        return self._state

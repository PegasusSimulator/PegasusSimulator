"""
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Simulates a magnetometer. Based on the original implementation provided in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo) by Elia Tarasov
"""
__all__ = ["Magnetometer"]

#import numpy as np
import torch

#from scipy.spatial.transform import Rotation
import pytorch3d.transforms as transforms

from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.rotations import rot_ENU_to_NED, rot_FLU_to_FRD
from pegasus.simulator.logic.sensors.geo_mag_utils import (
    get_mag_declination,
    get_mag_inclination,
    get_mag_strength,
    reprojection,
)

class Magnetometer(Sensor):
    """The class that implements a magnetometer sensor. This class inherits the base class Sensor.
    """

    def __init__(self, config={}, device="cpu"):
        """Initialize the Magnetometer class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the Magnetometer - it can be empty or only have some of the parameters used by the Magnetometer.
            
        Examples:
            The dictionary default parameters are

            >>> {"noise_density": 0.4e-3,           # gauss / sqrt(hz)
            >>>  "random_walk": 6.4e-6,             # gauss * sqrt(hz)
            >>>  "bias_correlation_time": 6.0e2,    # s
            >>>  "update_rate": 250.0}              # Hz
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="Magnetometer", update_rate=torch.tensor(config.get("update_rate", 250.0), dtype=torch.float32, device=device))

        # Set device
        self.device = device
        
        # Set the noise parameters
        self._bias: torch.Tensor = torch.zeros(3, dtype=torch.float32, device=self.device)
        self._noise_density = torch.tensor(config.get("noise_density", 0.4e-3), dtype=torch.float32, device=self.device)  # gauss / sqrt(hz)
        self._random_walk = torch.tensor(config.get("random_walk", 6.4e-6), dtype=torch.float32, device=self.device)  # gauss * sqrt(hz)
        self._bias_correlation_time = torch.tensor(config.get("bias_correlation_time", 6.0e2), dtype=torch.float32, device=self.device)  # s

        # Initial state measured by the Magnetometer
        self._state = {"magnetic_field": torch.zeros(3, dtype=torch.float32, device=self.device)}

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state

    @Sensor.update_at_rate
    def update(self, state: State, dt: torch.Tensor):
        """Method that implements the logic of a magnetometer. In this method we start by computing the projection
        of the vehicle body frame such in the elipsoidal model of the earth in order to get its current latitude and 
        longitude. From here the declination and inclination are computed and used to get the strength of the magnetic
        field, expressed in the inertial frame of reference (in ENU convention). This magnetic field is then rotated 
        to the body frame such that it becomes expressed in a FRD body frame relative to a NED inertial reference frame.
        (The convention adopted by PX4). Random noise and bias are added to this magnetic field.

        Args:
            state (State): The current state of the vehicle.
            dt (torch.Tensor): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """
        # Get the latitude and longitude from the current state
        latitude, longitude = reprojection(state.position, torch.deg2rad(self._origin_lat), torch.deg2rad(self._origin_lon))

        # Magnetic declination and inclination (radians)
        declination_rad: torch.Tensor = torch.deg2rad(get_mag_declination(torch.rad2deg(latitude), torch.rad2deg(longitude)))
        inclination_rad: torch.Tensor = torch.deg2rad(get_mag_inclination(torch.rad2deg(latitude), torch.rad2deg(longitude)))

        # Compute the magnetic strength (10^5xnanoTesla)
        strength_ga: torch.Tensor = 0.01 * get_mag_strength(torch.rad2deg(latitude), torch.rad2deg(longitude))

        # Compute the Magnetic filed components according to: http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
        H: torch.Tensor = strength_ga * torch.cos(inclination_rad)
        Z: torch.Tensor = torch.tan(inclination_rad) * H
        X: torch.Tensor = H * torch.cos(declination_rad)
        Y: torch.Tensor = H * torch.sin(declination_rad)

        # Magnetic field of a body following a front-left-up (FLU) convention expressed in a East-North-Up (ENU) inertial frame
        magnetic_field_inertial: torch.Tensor = torch.stack([X, Y, Z]).to(dtype=torch.float32, device=self.device)

        # Rotate the magnetic field vector such that it expresses a field of a body frame according to the front-right-down (FRD)
        # expressed in a North-East-Down (NED) inertial frame (the standard used in magnetometer units)
        attitude_flu_enu = transforms.quaternion_to_matrix(state.attitude)

        # Rotate the magnetic field from the inertial frame to the body frame of reference according to the FLU frame convention
        rot_body_to_world = rot_ENU_to_NED(device=self.device, dtype=torch.float32) @ attitude_flu_enu @ rot_FLU_to_FRD(device=self.device, dtype=torch.float32).T

        # The magnetic field expressed in the body frame according to the front-right-down (FRD) convention
        magnetic_field_body = rot_body_to_world.T @ magnetic_field_inertial

        # -------------------------------
        # Add noise to the magnetic field
        # -------------------------------
        tau = self._bias_correlation_time

        # Discrete-time standard deviation equivalent to an "integrating" sampler with integration time dt.
        sigma_d: torch.Tensor = 1 / torch.sqrt(dt) * self._noise_density
        sigma_b: torch.Tensor = self._random_walk

        # Compute exact covariance of the process after dt [Maybeck 4-114].
        sigma_b_d: torch.Tensor = torch.sqrt(-sigma_b * sigma_b * tau / 2.0 * (torch.exp(-2.0 * dt / tau) - 1.0))

        # Compute state-transition.
        phi_d: torch.Tensor = torch.exp(-1.0 / tau * dt)

        # Add the noise to the magnetic field
        magnetic_field_noisy: torch.Tensor = torch.zeros((3,), device=self.device, dtype=torch.float32)
        for i in range(3):
            self._bias[i] = phi_d * self._bias[i] + sigma_b_d * torch.randn((), device=self.device, dtype=torch.float32)
            magnetic_field_noisy[i] = magnetic_field_body[i] + sigma_d * torch.randn((), device=self.device, dtype=torch.float32) + self._bias[i]

        # Add the values to the dictionary and return it
        self._state = {"magnetic_field": magnetic_field_noisy}

        return self._state

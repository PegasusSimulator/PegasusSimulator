"""
| File: imu.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Simulates a monocular camera attached to the vehicle
"""
__all__ = ["MonocularCamera"]

from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.graphical_sensors import GraphicalSensor

# Camera interface provided by NVidia Isaac Sim
from omni.isaac.sensor import Camera

# Auxiliary scipy and numpy modules
import numpy as np
from scipy.spatial.transform import Rotation


class MonocularCamera(GraphicalSensor):
    """
    The class that implements a monocular camera sensor. This class inherits the base class GraphicalSensor.
    """

    def __init__(self, config={}):
        """Initialize the MonocularCamera class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the MonocularCamera - it can be empty or only have some of the parameters used by the MonocularCamera.

        Examples:
            The dictionary default parameters are

            >>> {"focal_length": 0.004,
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="MonocularCamera", update_rate=config.get("update_rate", 60.0))

        self._state = {}
        
        # Create the actual camera object attached to the rigid body vehicle
        

    def initialize(self, vehicle):
        
        # Initialize the Super class "object" attributes
        super().initialize(vehicle)

        # Create the camera object
        self._camera = Camera(
            prim_path="/World/camera",
            position=np.array([0.0, 0.0, 25.0]),
            frequency=20,
            resolution=(256, 256),
            orientation=Rotation.from_euler("XYZ", [0.0, 90.0, 0.0], degrees=True).as_quat())

    def start(self):

        # Start the camera
        self._camera.initialize()

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state

    @GraphicalSensor.update_at_rate
    def update(self, state: State, dt: float):
        """Method that implements the logic of an IMU. In this method we start by generating the random walk of the 
        gyroscope. This value is then added to the real angular velocity of the vehicle (FLU relative to ENU inertial frame
        expressed in FLU body frame). The same logic is followed for the accelerometer and the accelerations. After this step,
        the angular velocity is rotated such that it expressed a FRD body frame, relative to a NED inertial frame, expressed
        in the FRD body frame. Additionally, the acceleration is also rotated, such that it becomes expressed in the body
        FRD frame of the vehicle. This sensor outputs data that follows the PX4 adopted standard.

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        return self._state

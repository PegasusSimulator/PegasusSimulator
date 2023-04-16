"""
| File: rgb_camera.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Simulates an RGB camera.
"""
__all__ = ["RGBCamera"]

import numpy as np

# Import the Isaac Sim Camera API
from omni.isaac.sensor import Camera

from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors import Sensor


class RGBCamera(Sensor):
    """The class that implements the Camera sensor. This class inherits the base class Sensor.
    """
    def __init__(self, id=0, config={}):
        """Initialize the Camera class

        Args:
            id (int): The id of the camera
            config (dict): A Dictionary that contains all the parameters for configuring the IMU - it can be empty or only have some of the parameters used by the IMU.

        Examples:
            The dictionary default parameters are

            >>> {"position": [0.0, 0.0, 0.0],         # Meters
            >>>  "orientation": [0.0, 0.0, 0.0, 1.0], # Quaternion [qx, qy, qz, qw]
            >>>  "focal_length": 250.0,               # Pixels
            >>>  "resolution": [640, 480],            # Pixels
            >>>  "set_projection_type": "pinhole",    # pinhole, fisheyeOrthographic, fisheyeEquidistant, fisheyeEquisolid, fisheyePolynomial, fisheyeSpherical
            >>>  "update_rate": 60.0}                 # Hz
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="RGBCamera", update_rate=config.get("update_rate", 60.0))

        # Save the id of the sensor
        self._id = id

        # Reference to the actual camera object. This is set when the camera is initialized
        self._vehicle = None                # The vehicle this sensor is associated with
        self.camera = None

        # Set the position of the camera relative to the vehicle
        self._position = np.array(config.get("position", [0.0, 0.0, 0.0]))
        self._orientation = np.array(config.get("orientation", [0.0, 0.0, 0.0, 1.0]))  # Quaternion [qx, qy, qz, qw]

        # Set the camera parameters
        self._focal_length = config.get("focal_length", 250.0)
        self._resolution = config.get("resolution", [640, 480])
        self._set_projection_type = config.get("set_projection_type", "pinhole")

        # Save the current state of the camera sensor
        self._state = {
            "id": self._id,
            "position": np.array([0.0, 0.0, 0.0]),
            "orientation": np.array([0.0, 0.0, 0.0, 1.0]),
            "frame_num": 0,
            "frame": None,
        }

    def initialize(self, origin_lat, origin_lon, origin_alt, vehicle=None):
        """Method that initializes the sensor latitude, longitude and altitude attributes as well 
        as the vehicle that the sensor is attached to.
        
        Args:
            origin_lat (float): NOT USED BY THIS SENSOR
            origin_lon (float): NOT USED BY THIS SENSOR
            origin_alt (float): NOT USED BY THIS SENSOR
            vehicle (Vehicle): The vehicle that this sensor is attached to.
        """

        self._vehicle = vehicle

        # Set the prim_path for the camera
        camera_prim_path = "/World/camera" + str(self._id) if self._vehicle is None else self._vehicle.prim_path + "/body/camera" + str(self._id)

        # Create the actual camera object
        self.camera = Camera(
            prim_path=camera_prim_path,
            position=self._position,
            frequency=self._update_rate,
            resolution=self._resolution,
            orientation=self._orientation
        )

        # Initialize the camera sensor
        self.camera.initialize()

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state

    @Sensor.update_at_rate
    def update(self, state: State, dt: float):
        """

        Args:
            state (State): The current state of the vehicle. UNUSED IN THIS SENSOR
            dt (float): The time elapsed between the previous and current function calls (s). UNUSED IN THIS SENSOR

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        # Get the most up to date frame from the camera
        data = self.camera.get_current_frame()

        # Get is absolute position and orientation relative to the inertial frame in ENU
        pos, quat = self.camera.get_world_pose()

        # Update the state object
        self._state = {
            "id": self._id,
            "position": pos,
            "orientation": np.array([quat[1], quat[2], quat[3], quat[0]]),
            "frame_num": data.get('rendering_frame', -1),
            "frame": data.get('rgba', None)
        }

        return self._state

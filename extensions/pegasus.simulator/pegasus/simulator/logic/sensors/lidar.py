"""
| File: lidar.py
| License: BSD-3-Clause. Copyright (c) 2023, Micah Nye. All rights reserved.
| Description: Creates a lidar sensor
"""
__all__ = ["Lidar"]

from omni.usd import get_context
from omni.isaac.range_sensor._range_sensor import acquire_lidar_sensor_interface
import omni.isaac.RangeSensorSchema as RangeSensorSchema
from pxr import Sdf, Gf

from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.vehicles import Vehicle
import numpy as np

class Lidar(Sensor):
    """The class that implements the Lidar sensor. This class inherits the base class Sensor.
    """
    def __init__(self, prim_path: str, config: dict = {}):
        """Initialize the Lidar class
        Args:
            prim_path (str): Path to the lidar prim. Global path when it starts with `/`, else local to vehicle prim path
            config (dict): A Dictionary that contains all the parameters for configuring the lidar - it can be empty or only have some of the parameters used by the lidar.
        Examples:
            The dictionary default parameters are
            >>> {"position": [0.0, 0.0, 0.0],           # Meters
            >>>  "yaw_offset": 0.0,                     # Degrees
            >>>  "rotation_rate": 20.0,                 # Hz
            >>>  "horizontal_fov": 360.0,               # Degrees
            >>>  "horizontal_resolution": 1.0,          # Degrees
            >>>  "vertical_fov": 10.0,                  # Degrees
            >>>  "vertical_resolution": 1.0,            # Degrees
            >>>  "min_range": 0.4,                      # Meters
            >>>  "max_range": 100.0,                    # Meters
            >>>  "high_lod": True,                      # High level of detail (True - draw all rays, False - draw horizontal rays)
            >>>  "draw_points": False,                  # Draw lidar points where they hit an object
            >>>  "draw_lines": False,                   # Draw lidar ray lines
            >>>  "fill_state: False}                    # Fill state with sensor data
        """

        # Initialize the Super class "object" attribute
        # update_rate not necessary
        super().__init__(sensor_type="Lidar", update_rate=config.get("rotation_rate", 20.0))

        # Save the id of the sensor
        self._prim_path = prim_path
        self._frame_id = prim_path.rpartition("/")[-1] # frame_id of the camera is the last prim path part after `/`

        # The extension acquires the LIDAR interface at startup.  It will be released during extension shutdown.  We
        # create a LIDAR prim using our schema, and then we interact with / query that prim using the python API found
        # in lidar/bindings
        self._li = acquire_lidar_sensor_interface()
        self.lidar = None

        # Get the lidar position relative to its parent prim
        self._position = np.array(config.get("position", [0.0, 0.0, 0.0]))

        # Get the lidar parameters
        self._yaw_offset = config.get("yaw_offset", 0.0)
        self._rotation_rate = config.get("rotation_rate", 20.0)
        self._horizontal_fov = config.get("horizontal_fov", 360.0)
        self._horizontal_resolution = config.get("horizontal_resolution", 1.0)
        self._vertical_fov = config.get("vertical_fov", 10.0)
        self._vertical_resolution = config.get("vertical_resolution", 1.0)
        self._min_range = config.get("min_range", 0.4)
        self._max_range = config.get("max_range", 100.0)
        self._high_lod = config.get("high_lod", True)
        self._draw_points = config.get("draw_points", False)
        self._draw_lines = config.get("draw_lines", False)

        # Save the current state of the range sensor
        self._fill_state = config.get("fill_state", False)
        if self._fill_state:
            self._state = {
                "frame_id": self._frame_id,
                "depth": None,
                "zenith": None,
                "azimuth": None
            }
        else:
            self._state = None

    def initialize(self, vehicle: Vehicle, origin_lat, origin_lon, origin_alt):
        """Method that initializes the lidar sensor. It also initalizes the sensor latitude, longitude and
        altitude attributes as well as the vehicle that the sensor is attached to.
        
        Args:
            vehicle (Vehicle): The vehicle that this sensor is attached to.
            origin_lat (float): The latitude of the origin of the world in degrees (might get used by some sensors).
            origin_lon (float): The longitude of the origin of the world in degrees (might get used by some sensors).
            origin_alt (float): The altitude of the origin of the world relative to sea water level (might get used by some sensors).
        """
        super().initialize(vehicle, origin_lat, origin_lon, origin_alt)

        # Set the prim path for the camera
        if self._prim_path[0] != '/':
            self._prim_path = f"{vehicle.prim_path}/{self._prim_path}"
        else:
            self._prim_path = self._prim_path
        

        # create the LIDAR.  Before we can set any attributes on our LIDAR, we must first create the prim using our
        # LIDAR schema, and then populate it with the parameters we will be manipulating.  If you try to manipulate
        # a parameter before creating it, you will get a runtime error
        stage = get_context().get_stage()
        self.lidar = RangeSensorSchema.Lidar.Define(stage, Sdf.Path(self._prim_path))

        # Set lidar parameters
        self.lidar.AddTranslateOp().Set(Gf.Vec3f(*self._position))
        self.lidar.CreateYawOffsetAttr().Set(self._yaw_offset)
        self.lidar.CreateRotationRateAttr().Set(self._rotation_rate)
        self.lidar.CreateHorizontalFovAttr().Set(self._horizontal_fov)
        self.lidar.CreateHorizontalResolutionAttr().Set(self._horizontal_resolution)
        self.lidar.CreateVerticalFovAttr().Set(self._vertical_fov)
        self.lidar.CreateVerticalResolutionAttr().Set(self._vertical_resolution)
        self.lidar.CreateMinRangeAttr().Set(self._min_range)
        self.lidar.CreateMaxRangeAttr().Set(self._max_range)
        self.lidar.CreateHighLodAttr().Set(self._high_lod)
        self.lidar.CreateDrawPointsAttr().Set(self._draw_points)
        self.lidar.CreateDrawLinesAttr().Set(self._draw_lines)
    
        # Set the sensor's frame path
        self.frame_path = self._prim_path

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
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).
        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor) or None
        """

        # Add the values to the dictionary and return it
        if self._fill_state:
            self._state = {
                "frame_id": self._frame_id,
                "depth": self._li.get_depth_data(self._prim_path),
                "zenith": self._li.get_zenith_data(self._prim_path),
                "azimuth": self._li.get_azimuth_data(self._prim_path),
            }

        return self._state

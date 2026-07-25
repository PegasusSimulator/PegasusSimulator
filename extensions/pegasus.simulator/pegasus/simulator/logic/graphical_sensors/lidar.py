"""
| File: lidar.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Simulates a lidar attached to the vehicle
"""
__all__ = ["Lidar"]

from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.graphical_sensors import GraphicalSensor
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Imports the python bindings to interact with lidar sensor
import omni.kit.commands
from pxr import Gf, UsdGeom  
from omni.usd import get_stage_next_free_path

# Auxiliary scipy and numpy modules
import numpy as np
from scipy.spatial.transform import Rotation

# Import the replicatore core module used for writing graphical data to ROS 2
import omni.replicator.core as rep

class Lidar(GraphicalSensor):

    def __init__(self, lidar_name, config={}):
        """
        Initialize the Lidar class

        The available configurations for the lidar are available in the folder exts/omni.isaac.sensor/data/lidar_configs
        Check also the official documentation:
        https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar.html
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="Lidar", update_rate=config.get("frequency", 60.0)) 

        # Setup the name of the camera primitive path
        self._lidar_name = lidar_name
        self._stage_prim_path = ""

        # Configurations of the Lidar
        self._position = config.get("position", np.array([0.0, 0.0, 0.10]))
        self._orientation = Rotation.from_euler("ZYX", config.get("orientation", np.array([0.0, 0.0, 0.0])), degrees=True).as_quat()
        self._sensor_configuration = config.get("sensor_configuration", "Example_Rotary")
        self._sensor_attributes = config.get("sensor_attributes", {})
        self._show_render = config.get("show_render", False)

        # If True publish a full scan when enough data has accumulated instead of partial scans each frame. Supports point cloud type only
        self._full_scan = config.get("full_scan", False)

        self._sensor = None

    def initialize(self, vehicle):
        """
        Initialize the Lidar sensor
        """
        super().initialize(vehicle)
        
        # Get the complete stage prefix for the lidar
        self._stage_prim_path = get_stage_next_free_path(PegasusInterface().world.stage, self._vehicle.prim_path + "/body/" + self._lidar_name, False)

        # Get the camera name that was actually created (and update the camera name)
        self._lidar_name = self._stage_prim_path.rpartition("/")[-1]
        
        _, self._sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=self._lidar_name,
            parent=self._vehicle.prim_path + "/body",
            config= self._sensor_configuration,
            translation=(self._position[0], self._position[1], self._position[2]),
            orientation=Gf.Quatd(self._orientation[3], self._orientation[0], self._orientation[1], self._orientation[2]),
            **self._sensor_attributes
        )
        self._stage_prim_path = self._sensor.GetPath()
        self._number_of_emitters = self._sensor.GetAttribute("omni:sensor:Core:numberOfEmitters").Get()
    
    def start(self):
        # If show_render is True, then create a render product for the lidar in the Isaac Sim environment
        if self._show_render:
            hydra_texture = rep.create.render_product(self._sensor.GetPath(), [1, 1], name=self._lidar_name)
            writer = rep.writers.get("RtxLidarDebugDrawPointCloud" + ("Buffer" if not self._full_scan else ""))
            writer.initialize(doTransform=True)
            writer.attach([hydra_texture])

    @property
    def name(self):
        """
        (str) The name of the lidar sensor.
        """
        return self._lidar_name

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state
    
    @GraphicalSensor.update_at_rate
    def update(self, state: State, dt: float):
        """Method that gets the data from the lidar and returns it as a dictionary.

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        # Just return the prim path and the name of the lidar
        self._state = {
            "lidar_name": self._lidar_name,
            "stage_prim_path": self._stage_prim_path,
            "full_scan": self._full_scan,
            "number_of_emitters": self._number_of_emitters,
        }

        return self._state
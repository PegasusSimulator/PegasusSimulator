"""
| File: ros2_lidar.py
| License: BSD-3-Clause. Copyright (c) 2023, Micah Nye. All rights reserved.
"""
__all__ = ["ROS2LidarGraph"]

import carb

import omni.graph.core as og
from omni.isaac.core.utils.prims import is_prim_path_valid

from pegasus.simulator.logic.graphs import Graph
from pegasus.simulator.logic.vehicles import Vehicle

class ROS2LidarGraph(Graph):
    """The class that implements the ROS2 Lidar graph. This class inherits the base class Graph.
    """
    def __init__(self, lidar_prim_path: str, config: dict = {}):
        """Initialize the ROS2 Lidar class

        Args:
            lidar_prim_path (str): Path to the lidar prim. Global path when it starts with `/`, else local to vehicle prim path
            config (dict): A Dictionary that contains all the parameters for configuring the ROS2Lidar - it can be empty or only have some of the parameters used by the ROS2Lidar.

        Examples:
            The dictionary default parameters are

            >>> {"publish_scan": False,                     # publish scanner data as sensor_msgs/LaserScan (requires high_lod turned off)
            >>>  "publish_point_cloud": True}               # publish scanner data as sensor_msgs/PointCloud2 (for 2D data, requires high_lod turned on)

        Note:
            To publish scan data, HighLOD needs to be turned off on the lidar prim. This means that only 2D laser scans are supported.
        """

        # Initialize the Super class "object" attribute
        super().__init__(graph_type="ROS2LidarGraph")

        # Save lidar path, frame id and ros topic name
        self._lidar_prim_path = lidar_prim_path
        self._frame_id = lidar_prim_path.rpartition("/")[-1] # frame_id of the lidar is the last prim path part after `/`
        self._base_topic = ""

        # Process the config dictionary
        self._publish_scan = config.get("publish_scan", False)
        self._publish_point_cloud = config.get("publish_point_cloud", True)

    def initialize(self, vehicle: Vehicle):
        """Method that initializes the graph of the lidar.

        Args:
            vehicle (Vehicle): The vehicle that this graph is attached to.
        """

        self._namespace = f"/{vehicle.vehicle_name}"
        self._base_topic = f"/{self._frame_id}"

        # Set the prim_path for the camera
        if self._lidar_prim_path[0] != '/':
            self._lidar_prim_path = f"{vehicle.prim_path}/{self._lidar_prim_path}"

        # Check if the prim path is valid
        if not is_prim_path_valid(self._lidar_prim_path):
            carb.log_error(f"Cannot create ROS2 Lidar graph, the lidar prim path \"{self._lidar_prim_path}\" is not valid")
            return
        
        # Set the prim paths for camera and tf graphs
        graph_path = f"{self._lidar_prim_path}_pub"

        # Graph configuration
        graph_specs = {
            "graph_path": graph_path,
            "evaluator_name": "execution",
        }

        # Creating a default graph edit configuration
        keys = og.Controller.Keys
        graph_config = {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnTick"),
                ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            ],
            keys.CONNECT: [],
            keys.SET_VALUES: [],
        }

        # Add laser scan publishing to the graph
        if self._publish_scan:

            graph_config[keys.CREATE_NODES] += [
                ("isaac_read_lidar_beams", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                ("publish_laser_scan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan")
            ]
            graph_config[keys.CONNECT] += [
                ("on_tick.outputs:tick", "isaac_read_lidar_beams.inputs:execIn"),
                ("isaac_read_lidar_beams.outputs:execOut", "publish_laser_scan.inputs:execIn"),
                ("isaac_read_lidar_beams.outputs:azimuthRange", "publish_laser_scan.inputs:azimuthRange"),
                ("isaac_read_lidar_beams.outputs:depthRange", "publish_laser_scan.inputs:depthRange"),
                ("isaac_read_lidar_beams.outputs:horizontalFov", "publish_laser_scan.inputs:horizontalFov"),
                ("isaac_read_lidar_beams.outputs:horizontalResolution", "publish_laser_scan.inputs:horizontalResolution"),
                ("isaac_read_lidar_beams.outputs:intensitiesData", "publish_laser_scan.inputs:intensitiesData"),
                ("isaac_read_lidar_beams.outputs:linearDepthData", "publish_laser_scan.inputs:linearDepthData"),
                ("isaac_read_lidar_beams.outputs:numCols", "publish_laser_scan.inputs:numCols"),
                ("isaac_read_lidar_beams.outputs:numRows", "publish_laser_scan.inputs:numRows"),
                ("isaac_read_lidar_beams.outputs:rotationRate", "publish_laser_scan.inputs:rotationRate"),
                ("isaac_read_simulation_time.outputs:simulationTime", "publish_laser_scan.inputs:timeStamp")
            ]
            graph_config[keys.SET_VALUES] += [
                ("isaac_read_lidar_beams.inputs:lidarPrim", self._lidar_prim_path),
                ("publish_laser_scan.inputs:frameId", self._frame_id),
                ("publish_laser_scan.inputs:nodeNamespace", self._namespace),
                ("publish_laser_scan.inputs:topicName", f"{self._base_topic}/scan")
            ]

        # Add point cloud publishing to the graph
        if self._publish_point_cloud:
            graph_config[keys.CREATE_NODES] += [
                ("isaac_read_lidar_point_cloud", "omni.isaac.range_sensor.IsaacReadLidarPointCloud"),
                ("publish_point_cloud", "omni.isaac.ros2_bridge.ROS2PublishPointCloud")
            ]
            graph_config[keys.CONNECT] += [
                ("on_tick.outputs:tick", "isaac_read_lidar_point_cloud.inputs:execIn"),
                ("isaac_read_lidar_point_cloud.outputs:execOut", "publish_point_cloud.inputs:execIn"),
                ("isaac_read_lidar_point_cloud.outputs:data", "publish_point_cloud.inputs:data"),
                ("isaac_read_simulation_time.outputs:simulationTime", "publish_point_cloud.inputs:timeStamp")
            ]
            graph_config[keys.SET_VALUES] += [
                ("isaac_read_lidar_point_cloud.inputs:lidarPrim", self._lidar_prim_path),
                ("publish_point_cloud.inputs:frameId", self._frame_id),
                ("publish_point_cloud.inputs:nodeNamespace", self._namespace),
                ("publish_point_cloud.inputs:topicName", f"{self._base_topic}/point_cloud")
            ]
        
        # Create the lidar graph
        (graph, _, _, _) = og.Controller.edit(
            graph_specs,
            graph_config
        )
        
        # Run the ROS Lidar graph once to generate ROS publishers in SDGPipeline
        og.Controller.evaluate_sync(graph)

        # Also initialize the Super class with updated prim path (only lidar graph path)
        super().initialize(graph_path)

    def laser_scan_topic(self) -> str:
        """
        Returns:
            (str) Lidar laser scan topic name if exists, else empty string
        """
        return f"{self._namespace}{self._base_topic}/scan" if self._publish_scan else ""

    def point_cloud_topic(self) -> str:
        """
        Returns:
            (str) Lidar point cloud topic name if exists, else empty string
        """
        return f"{self._namespace}{self._base_topic}/point_cloud" if self._publish_point_cloud else ""
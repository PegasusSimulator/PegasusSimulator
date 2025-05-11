"""
| File: ros2_odometry.py
| License: BSD-3-Clause. Copyright (c) 2023, Micah Nye. All rights reserved.
"""
__all__ = ["ROS2OdometryGraph"]

import carb

from omni.isaac.core.utils import stage
import omni.graph.core as og
from omni.isaac.core.utils.prims import is_prim_path_valid, set_targets

from pegasus.simulator.logic.graphs import Graph
from pegasus.simulator.logic.vehicles import Vehicle


class ROS2OdometryGraph(Graph):
    """The class that implements the ROS2 Odometry graph. This class inherits the base class Graph."""

    def __init__(self, config: dict = {}):
        """Initialize the ROS2 Odometry class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the ROS2Odometry - it can be
            empty or only have some of the parameters used by the ROS2Odometry.

        Examples:
            The dictionary default parameters are

            >>> {"odom_topic": "odom",                          # String for odometry topic
            >>>  "publish_odom_to_base_tf": True,               # Enable tf broadcaster for odom_frame->base_frame transform
            >>>  "publish_map_to_odom_tf": True,                # Enable tf broadcaster for map_frame->odom_frame transform
            >>>  "map_frame": "map",                            # String name for the map_frame
            >>>  "odom_frame": "odom",                          # String name for the odom_frame
            >>>  "base_frame": "base_link"}                     # String name for the base_frame
        """

        # Initialize the Super class "object" attribute
        super().__init__(graph_type="ROS2OdometryGraph")

        # Process the config dictionary
        self._odom_topic = config.get("odom_topic", "odom")
        self._publish_odom_to_base_tf = config.get("publish_map_to_odom_tf", True)
        self._publish_map_to_odom_tf = config.get("publish_map_to_odom_tf", True)
        self._map_frame = config.get("map_frame", "map")
        self._odom_frame = config.get("odom_frame", "odom")
        self._base_frame = config.get("base_frame", "base_link")

    def initialize(self, vehicle: Vehicle):
        """Method that initializes the graph.

        Args:
            vehicle (Vehicle): The vehicle that this graph is attached to.
        """

        self._namespace = f"/{vehicle.vehicle_name}"

        # Create the graph under vehicle with graph name odom_pub and allow only one per vehicle.
        graph_path = f"{vehicle.prim_path}/odom_pub"
        if is_prim_path_valid(graph_path):
            carb.log_warn(f"ROS2 Odometry Graph for vehicle {vehicle.vehicle_name} already exists")
            return

        # Graph configuration
        graph_specs = {
            "graph_path": graph_path,
            "evaluator_name": "execution",
        }

        # Creating a graph edit configuration with transform tree publishers
        keys = og.Controller.Keys
        graph_config = {
            keys.CREATE_NODES: [
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("isaac_compute_odometry", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                ("publish_odometry", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
            ],
            keys.CONNECT: [
                ("on_playback_tick.outputs:tick", "isaac_compute_odometry.inputs:execIn"),
                ("isaac_read_simulation_time.outputs:simulationTime", "publish_odometry.inputs:timeStamp"),
                ("isaac_compute_odometry.outputs:execOut", "publish_odometry.inputs:execIn"),
                ("isaac_compute_odometry.outputs:linearVelocity", "publish_odometry.inputs:linearVelocity"),
                ("isaac_compute_odometry.outputs:orientation", "publish_odometry.inputs:orientation"),
                ("isaac_compute_odometry.outputs:position", "publish_odometry.inputs:position"),
            ],
            keys.SET_VALUES: [
                ("publish_odometry.inputs:odomFrameId", self._odom_frame),
                ("publish_odometry.inputs:chassisFrameId", self._base_frame),
                ("publish_odometry.inputs:nodeNamespace", self._namespace),
                ("publish_odometry.inputs:topicName", self._odom_topic),
            ],
        }

        # Create odom_frame->base_frame publisher
        if self._publish_odom_to_base_tf:
            graph_config[keys.CREATE_NODES] += [
                ("publish_odom_transform_tree", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree")
            ]
            graph_config[keys.CONNECT] += [
                ("on_playback_tick.outputs:tick", "publish_odom_transform_tree.inputs:execIn"),
                ("isaac_read_simulation_time.outputs:simulationTime", "publish_odom_transform_tree.inputs:timeStamp"),
                ("isaac_compute_odometry.outputs:orientation", "publish_odom_transform_tree.inputs:rotation"),
                ("isaac_compute_odometry.outputs:position", "publish_odom_transform_tree.inputs:translation"),
            ]
            graph_config[keys.SET_VALUES] += [
                ("publish_odom_transform_tree.inputs:parentFrameId", self._odom_frame),
                ("publish_odom_transform_tree.inputs:childFrameId", self._base_frame),
                ("publish_odom_transform_tree.inputs:nodeNamespace", self._namespace),
            ]

        # Create map_frame->odom_frame publisher
        # Because there is no drift or pose jumps in simulated odometry, map_frame->base_frame == odom_frame->base_frame
        if self._publish_odom_to_base_tf:
            graph_config[keys.CREATE_NODES] += [
                ("publish_map_transform_tree", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree")
            ]
            graph_config[keys.CONNECT] += [
                ("on_playback_tick.outputs:tick", "publish_map_transform_tree.inputs:execIn"),
                ("isaac_read_simulation_time.outputs:simulationTime", "publish_map_transform_tree.inputs:timeStamp"),
            ]
            graph_config[keys.SET_VALUES] += [
                ("publish_map_transform_tree.inputs:parentFrameId", self._map_frame),
                ("publish_map_transform_tree.inputs:childFrameId", self._odom_frame),
                ("publish_map_transform_tree.inputs:nodeNamespace", self._namespace),
            ]

        # Create the camera graph
        (graph, _, _, _) = og.Controller.edit(graph_specs, graph_config)

        # Set the odometry chassis prim, which should be the vehicle prim path
        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(f"{graph_path}/isaac_compute_odometry"),
            attribute="inputs:chassisPrim",
            target_prim_paths=[vehicle.prim_path],
        )

        # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
        og.Controller.evaluate_sync(graph)

        # Also initialize the Super class with updated prim path (only camera graph path)
        super().initialize(graph_path)

    @property
    def odometry_topic(self) -> str:
        """
        (str) Path to the odometry topic.

        Returns:
            Odometry topic name (str)
        """
        return f"{self._namespace}/{self._odom_topic}"

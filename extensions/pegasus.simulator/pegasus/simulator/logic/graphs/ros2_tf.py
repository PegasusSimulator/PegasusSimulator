"""
| File: ros2_tf.py
| License: BSD-3-Clause. Copyright (c) 2023, Micah Nye. All rights reserved.
"""
__all__ = ["ROS2Tf"]

import carb

from omni.isaac.core.utils import stage
import omni.graph.core as og
from omni.isaac.core.utils.prims import is_prim_path_valid, set_targets
from omni.isaac.core.prims import XFormPrim

from pegasus.simulator.logic.graphs import Graph
from pegasus.simulator.logic.vehicles import Vehicle


class ROS2Tf(Graph):
    """The class that implements the ROS2 TF graph. This class inherits the base class Graph."""

    def __init__(self):
        """Initialize the ROS2 TF class"""

        # Initialize the Super class "object" attribute
        super().__init__(graph_type="ROS2Tf")

    def initialize(self, vehicle: Vehicle):
        """Method that initializes the graph.

        Args:
            vehicle (Vehicle): The vehicle that this graph is attached to.
        """

        self._namespace = f"/{vehicle.vehicle_name}"

        # The vehicle uses body instead of standardized base_link,
        # so we need to create the base_link and connect the body to it
        base_link_xform_path = f"{vehicle.prim_path}/body/base_link"
        XFormPrim(prim_path=base_link_xform_path)

        # Create the graph under vehicle with graph name tf and allow only one per vehicle.
        graph_path = f"{vehicle.prim_path}/tf_pub"
        if is_prim_path_valid(graph_path):
            carb.log_warn(f"ROS2 TF Graph for vehicle {vehicle.vehicle_name} already exists")
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
                ("publish_transform_tree", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("on_playback_tick.outputs:tick", "publish_transform_tree.inputs:execIn"),
                ("isaac_read_simulation_time.outputs:simulationTime", "publish_transform_tree.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("publish_transform_tree.inputs:nodeNamespace", self._namespace),
                ("publish_transform_tree.inputs:parentPrim", base_link_xform_path)
            ],
        }

        # Create list of target prims, which will contain articulation root
        # and all sensors with frame_path filled
        target_prim_paths = [vehicle.prim_path]

        for sensor in vehicle._sensors:
            if hasattr(sensor, "frame_path"):
                if isinstance(sensor.frame_path, list):
                    for frame_path in sensor.frame_path:
                        if isinstance(frame_path, list) and len(frame_path) > 1:
                            # frame_path[0] is parent for frame_path[1:]
                            base_frame_id = frame_path[0].rpartition("/")[-1]
                            extra_paths = []
                            for path in frame_path[1:]:
                                if is_prim_path_valid(path):
                                    extra_paths.append(path)

                            graph_config[keys.CREATE_NODES] += [
                                (base_frame_id + "_pub_tf_tree", "omni.isaac.ros2_bridge.ROS2PublishTransformTree")
                            ]
                            graph_config[keys.CONNECT] += [
                                ("on_playback_tick.outputs:tick", base_frame_id + "_pub_tf_tree.inputs:execIn"),
                                ("isaac_read_simulation_time.outputs:simulationTime",
                                 base_frame_id + "_pub_tf_tree.inputs:timeStamp"),
                            ]
                            graph_config[keys.SET_VALUES] += [
                                (base_frame_id + "_pub_tf_tree.inputs:nodeNamespace", self._namespace),
                                (base_frame_id + "_pub_tf_tree.inputs:parentPrim", frame_path[0]),
                                (base_frame_id + "_pub_tf_tree.inputs:targetPrims", extra_paths)
                            ]
                        else:
                            if len(frame_path) and is_prim_path_valid(frame_path):
                                target_prim_paths.append(frame_path)
                else:
                    if len(sensor.frame_path) and is_prim_path_valid(sensor.frame_path):
                        target_prim_paths.append(sensor.frame_path)

        graph_config[keys.SET_VALUES] += [
            ("publish_transform_tree.inputs:targetPrims", target_prim_paths)
        ]

        # Create the tf graph
        (graph, _, _, _) = og.Controller.edit(graph_specs, graph_config)

        # Run the ROS Tf graph once to generate ROS tf publishers in SDGPipeline
        og.Controller.evaluate_sync(graph)

        # Also initialize the Super class with updated prim path
        super().initialize(graph_path)




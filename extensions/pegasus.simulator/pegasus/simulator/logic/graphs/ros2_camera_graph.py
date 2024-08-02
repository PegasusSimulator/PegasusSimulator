"""
| File: ros2_camera.py
| License: BSD-3-Clause. Copyright (c) 2023, Micah Nye. All rights reserved.
"""
__all__ = ["ROS2Camera"]

import carb

from omni.isaac.core.utils import stage
import omni.graph.core as og
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.prims import set_targets

from pegasus.simulator.logic.graphs import Graph
from pegasus.simulator.logic.vehicles import Vehicle
import numpy as np

class ROS2CameraGraph(Graph):
    """The class that implements the ROS2 Camera graph. This class inherits the base class Graph.
    """
    def __init__(self, camera_prim_path: str, config: dict = {}):
        """Initialize the ROS2 Camera class

        Args:
            camera_prim_path (str): Path to the camera prim. Global path when it starts with `/`, else local to vehicle prim path
            config (dict): A Dictionary that contains all the parameters for configuring the ROS2Camera - it can be empty or only have some of the parameters used by the ROS2Camera.

        Examples:
            The dictionary default parameters are

            >>> {"graph_evaluator": "execution",            # type of the omnigraph to create (execution, push)
            >>>  "resolution": [640, 480],                  # output video stream resolution in pixels [width, height]
            >>>  "types": ['rgb', 'camera_info'],           # rgb, depth, depth_pcl, instance_segmentation, semantic_segmentation, bbox_2d_tight, bbox_2d_loose, bbox_3d, camera_info
            >>>  "publish_labels": True,                    # publish labels for instance_segmentation, semantic_segmentation, bbox_2d_tight, bbox_2d_loose and bbox_3d camera types
            >>>  "topic": ""                                # base topic name for the camera (default is camera name in Isaac Sim)
            >>>  "namespace": ""                            # namespace for the camera (default is vehicle name in Isaac Sim)
            >>>  "tf_frame_id": ""}                         # tf frame id for the camera (default is camera name in Isaac Sim)
        """

        # Initialize the Super class "object" attribute
        super().__init__(graph_type="ROS2CameraGraph")

        # Save camera path, frame id and ros topic name
        self._camera_prim_path = camera_prim_path
        self._frame_id = camera_prim_path.rpartition("/")[-1] # frame_id of the camera is the last prim path part after `/`
        self._base_topic = config.get("topic", "")
        self._namespace = config.get("namespace", "")
        self._tf_frame_id = config.get("tf_frame_id", "")

        # Process the config dictionary
        self._graph_evaluator = config.get("graph_evaluator", "execution")
        self._resolution = config.get("resolution", [640, 480])
        self._types = np.array(config.get("types", ['rgb', 'camera_info']))
        self._publish_labels = config.get("publish_labels", True)

    def initialize(self, vehicle: Vehicle):
        """Method that initializes the graph of the camera.

        Args:
            vehicle (Vehicle): The vehicle that this graph is attached to.
        """

        # Set the namespace for the camera if non is provided
        if self._namespace == "":
            self._namespace = f"/{vehicle.vehicle_name}"

        # Set the base topic for the camera if non is provided
        if self._base_topic == "":
            self._base_topic = f"/{self._frame_id}"

        # Set the tf frame id for the camera if non is provided
        if self._tf_frame_id == "":
            self._tf_frame_id = self._frame_id

        # Set the prim_path for the camera
        if self._camera_prim_path[0] != '/':
            self._camera_prim_path = f"{vehicle.prim_path}/{self._camera_prim_path}"

        # Create camera prism
        if not is_prim_path_valid(self._camera_prim_path):
            carb.log_error(f"Cannot create ROS2 Camera graph, the camera prim path \"{self._camera_prim_path}\" is not valid")
            return

        # Set the prim paths for camera and tf graphs
        graph_path = f"{self._camera_prim_path}_pub"

        # Graph configuration
        if self._graph_evaluator == "execution":
            graph_specs = {
                "graph_path": graph_path,
                "evaluator_name": "execution",
            }
        elif self._graph_evaluator == "push":
            graph_specs = {
                "graph_path": graph_path,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            }
        else:
            carb.log_error(f"Cannot create ROS2 Camera graph, graph evaluator type \"{self._graph_evaluator}\" is not valid")
            return

        # Creating a graph edit configuration with cameraHelper nodes to generate ROS image publishers
        keys = og.Controller.Keys
        graph_config = {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnTick"),
                ("create_viewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                ("get_render_product", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                ("set_viewport_resolution", "omni.isaac.core_nodes.IsaacSetViewportResolution"),
                ("set_camera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "create_viewport.inputs:execIn"),
                ("create_viewport.outputs:execOut", "get_render_product.inputs:execIn"),
                ("create_viewport.outputs:viewport", "get_render_product.inputs:viewport"),
                ("create_viewport.outputs:execOut", "set_viewport_resolution.inputs:execIn"),
                ("create_viewport.outputs:viewport", "set_viewport_resolution.inputs:viewport"),
                ("set_viewport_resolution.outputs:execOut", "set_camera.inputs:execIn"),
                ("get_render_product.outputs:renderProductPath", "set_camera.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("create_viewport.inputs:viewportId", 0),
                ("create_viewport.inputs:name", f"{self._namespace}/{self._frame_id}"),
                ("set_viewport_resolution.inputs:width", self._resolution[0]),
                ("set_viewport_resolution.inputs:height", self._resolution[1]),
            ],
        }

        # Add camerasHelper for each selected camera type
        valid_camera_type = False
        for camera_type in self._types:
            if not camera_type in ["rgb", "depth", "depth_pcl", "semantic_segmentation", "instance_segmentation", "bbox_2d_tight", "bbox_2d_loose", "bbox_3d", "camera_info"]:
                continue

            camera_helper_name = f"camera_helper_{camera_type}"

            graph_config[keys.CREATE_NODES] += [
                (camera_helper_name, "omni.isaac.ros2_bridge.ROS2CameraHelper")
            ]
            graph_config[keys.CONNECT] += [
                ("set_camera.outputs:execOut", f"{camera_helper_name}.inputs:execIn"),
                ("get_render_product.outputs:renderProductPath", f"{camera_helper_name}.inputs:renderProductPath")
            ]
            graph_config[keys.SET_VALUES] += [
                (f"{camera_helper_name}.inputs:nodeNamespace", self._namespace),
                (f"{camera_helper_name}.inputs:frameId", self._tf_frame_id),
                (f"{camera_helper_name}.inputs:topicName", f"{self._base_topic}/{camera_type}"),
                (f"{camera_helper_name}.inputs:type", camera_type)
            ]

            # Publish labels for specific camera types
            if self._publish_labels and camera_type in ["semantic_segmentation", "instance_segmentation", "bbox_2d_tight", "bbox_2d_loose", "bbox_3d"]:
                graph_config[keys.SET_VALUES] += [
                    (camera_helper_name + ".inputs:enableSemanticLabels", True),
                    (camera_helper_name + ".inputs:semanticLabelsTopicName", f"{self._frame_id}/{camera_type}_labels")
                ]

            valid_camera_type = True

        if not valid_camera_type:
            carb.log_error(f"Cannot create ROS2 Camera graph, no valid camera type was selected")
            return

        # Create the camera graph
        (graph, _, _, _) = og.Controller.edit(
            graph_specs,
            graph_config
        )

        # Connect camera to the graphs
        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(f"{graph_path}/set_camera"),
            attribute="inputs:cameraPrim",
            target_prim_paths=[self._camera_prim_path]
        )

        # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
        og.Controller.evaluate_sync(graph)

        # Also initialize the Super class with updated prim path (only camera graph path)
        super().initialize(graph_path)

    def camera_topic(self, camera_type: str) -> str:
        """
        (str) Path to the camera topic.

        Args:
            camera_type (str): one of the supported camera output types

        Returns:
            Camera topic name (str) if the camera type exists, else empty string
        """
        return f"{self._namespace}{self._base_topic}/{camera_type}" if camera_type in self._types else ""

    def camera_labels_topic(self, camera_type: str) -> str:
        """
        (str) Path to the camera labels topic.

        Args:
            camera_type (str): one of the supported camera output types

        Returns:
            Camera labels topic name (str) if the camera type exists, else empty string
        """
        if not self._publish_labels or \
           not camera_type in self._types or \
           not camera_type in ["semantic_segmentation", "instance_segmentation", "bbox_2d_tight", "bbox_2d_loose", "bbox_3d"]:
            return ""

        return f"{self._namespace}{self._base_topic}/{camera_type}_labels"

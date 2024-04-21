"""
| File: ros1_graphs.py
| License: BSD-3-Clause. Copyright (c) 2024, Siky L. All rights reserved.
"""

__all__ = ["ROS1Camera", "ROS1TF", "ROS1Odom", "ROS1Clock"]

from typing import List
import numpy as np

import carb

import omni.graph.core as og
from omni.isaac.core.utils import stage
from omni.isaac.core.utils.prims import is_prim_path_valid, set_targets
from omni.isaac.core.utils.extensions import disable_extension, enable_extension
from omni.isaac.core.prims._impl.single_prim_wrapper import _SinglePrimWrapper

from pegasus.simulator.logic.graphs import Graph
from pegasus.simulator.logic.backends.utils import get_ros_extension


class ROS1Camera(Graph):
    """The class that implements the ROS1 Camera graph. This class inherits the base class Graph.
    Details: https://docs.omniverse.nvidia.com/isaacsim/latest/ros_tutorials/tutorial_ros_camera.html"""

    def __init__(
        self,
        camera_prim_path: str,
        types: List[str] = ["rgb", "depth", "camera_info"],
        resolution: List[int] = [640, 480],
        publish_labels: bool = True,
        frame_id: str = "",
        namespace: str = "",
        graph_evaluator="execution",
    ):
        """Initialize the ROS1 Camera class

        Args:
            camera_prim_path (str): Path to the camera prim. If the path starts with '/', it is a global path; otherwise, it is a local path relative to the target prim path.
            types (List[str]): List of camera output types. The default types are ["rgb", "depth", "camera_info"].
            resolution (List[int]): Resolution of the output video stream in pixels. The default resolution is [640, 480].
            publish_labels (bool): Flag indicating whether to publish labels for certain camera types. The default value is True.
            frame_id (str): Frame ID of the camera. The default value is an empty string.
            namespace (str): Namespace for the pub node. The default value is an empty string.
            graph_evaluator (str): Type of the omnigraph to create. Valid values are "execution" and "push". The default value is "execution".

        Note:
            The `camera_prim_path` parameter is the path to the camera prim. If the path starts with '/', it is a global path;
            otherwise, it is a local path relative to the target prim path which is generated in the initialize method.

            The `types` parameter can include the following camera output types:
            - "rgb": RGB image
            - "depth": Depth image
            - "depth_pcl": Point cloud from depth image
            - "semantic_segmentation": Semantic segmentation image
            - "instance_segmentation": Instance segmentation image
            - "bbox_2d_tight": Tight 2D bounding box image
            - "bbox_2d_loose": Loose 2D bounding box image
            - "bbox_3d": 3D bounding box image
            - "camera_info": Camera information

            The `resolution` parameter should be a list of two integers representing the width and height of the output video stream.

            The `publish_labels` parameter is applicable only to certain camera types: "semantic_segmentation",
            "instance_segmentation", "bbox_2d_tight", "bbox_2d_loose", and "bbox_3d". When set to True, labels will be
            published for these camera types.

            If the `frame_id` parameter is not provided, the last part of the camera path will be used as the frame ID.

            If the `namespace` parameter is not provided, the namespace will be generated based on the target prim name
            in the initialize method.

            The `graph_evaluator` parameter determines the type of omnigraph to create. "execution" creates a graph
            that is evaluated during execution, while "push" creates a graph that is evaluated on demand.

            The `graph_evaluator` parameter should be set to "execution" or "push". If an invalid value is provided,
            an error will be logged and the graph creation will fail.

            The topic names for the camera and labels are generated based on the frame ID.
            The camera topic name is in the format "/<namespace>/<frame_id>/<camera_type>",
            and the labels topic name is in the format "/<namespace>/<frame_id>/<camera_type>_labels".
        """

        # check ROS1 Bridge Extention
        if get_ros_extension() != "ros":
            carb.log_warn(f'disable ros2 extension: {disable_extension("omni.isaac.ros2_bridge")}')
            carb.log_warn(f'enable ros1 extension: {enable_extension("omni.isaac.ros_bridge")}')

        # Initialize the Super class "object" attribute
        super().__init__(graph_type="ROS1Camera")

        # Save camera path, frame id and ros topic name
        self._camera_prim_path = camera_prim_path
        self._frame_id = (
            self._camera_prim_path.rpartition("/")[-1] if frame_id == "" else frame_id
        )  # get the last part of the camera path as the frame id if frame_id param not provided
        self._base_topic = ""
        self._namespace = namespace

        # Process the config dictionary
        self._graph_evaluator = graph_evaluator
        self._resolution = resolution
        self._types = np.array(types)
        self._publish_labels = publish_labels

    def initialize(self, target_prim: _SinglePrimWrapper = None):
        """Method that initializes the graph of the camera.

        Args:
            target_prim (_SinglePrimWrapper): The target Prim to which this graph is attached, which should have `name` and `prim_path` attributes. The default value is None.

        Note:
            The `target_prim` parameter is the prim to which the graph is attached. If the parameter is not provided, the graph is attached to the root node.

            When `self._camera_prim_path` is a local path, the target prim is used to generate the full path to the camera prim; if the target prim is not provided, we will add a '/' to the beginning of the path.

            When `self._namespace` is not provided, tne namespace for the pub node is generated based on the target prim name. If the target prim is not provided, the namespace is an empty string.

            The base topic name is generated based on the frame ID. The camera topic name is in the format "/<namespace>/<frame_id>/<camera_type>", and the labels topic name is in the format "/<namespace>/<frame_id>/<camera_type>_labels".
        """
        # Set the namespace for the pub node
        if self._namespace == "":
            self._namespace = f"/{target_prim.name.lstrip('/')}" if target_prim is not None else ""
        self._base_topic = f"{self._frame_id}"  # change to relative topic name based on node namespace

        # Set the prim_path for the camera,
        if self._camera_prim_path[0] != "/":
            self._camera_prim_path = (
                f"{target_prim.prim_path}/{self._camera_prim_path}"
                if target_prim is not None
                else f"/{self._camera_prim_path}"
            )  # relative path

        # validate camera prism
        if not is_prim_path_valid(self._camera_prim_path):
            carb.log_error(
                f'Cannot create ROS1 Camera graph, the camera prim path "{self._camera_prim_path}" is not valid'
            )
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
            carb.log_error(
                f'Cannot create ROS1 Camera graph, graph evaluator type "{self._graph_evaluator}" is not valid'
            )
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
            if not camera_type in [
                "rgb",
                "depth",
                "depth_pcl",
                "semantic_segmentation",
                "instance_segmentation",
                "bbox_2d_tight",
                "bbox_2d_loose",
                "bbox_3d",
                "camera_info",
            ]:
                continue

            camera_helper_name = f"camera_helper_{camera_type}"

            graph_config[keys.CREATE_NODES] += [(camera_helper_name, "omni.isaac.ros_bridge.ROS1CameraHelper")]
            graph_config[keys.CONNECT] += [
                ("set_camera.outputs:execOut", f"{camera_helper_name}.inputs:execIn"),
                ("get_render_product.outputs:renderProductPath", f"{camera_helper_name}.inputs:renderProductPath"),
            ]
            graph_config[keys.SET_VALUES] += [
                (f"{camera_helper_name}.inputs:nodeNamespace", self._namespace),
                (f"{camera_helper_name}.inputs:frameId", self._frame_id),
                (f"{camera_helper_name}.inputs:topicName", f"{self._base_topic}/{camera_type}"),
                (f"{camera_helper_name}.inputs:type", camera_type),
            ]

            # Publish labels for specific camera types
            if self._publish_labels and camera_type in [
                "semantic_segmentation",
                "instance_segmentation",
                "bbox_2d_tight",
                "bbox_2d_loose",
                "bbox_3d",
            ]:
                graph_config[keys.SET_VALUES] += [
                    (camera_helper_name + ".inputs:enableSemanticLabels", True),
                    (
                        camera_helper_name + ".inputs:semanticLabelsTopicName",
                        f"{self._base_topic}/{camera_type}_labels",
                    ),
                ]

            valid_camera_type = True

        if not valid_camera_type:
            carb.log_error(f"Cannot create ROS1 Camera graph, no valid camera type was selected")
            return

        # Create the camera graph
        (graph, _, _, _) = og.Controller.edit(graph_specs, graph_config)

        # Connect camera to the graphs
        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(f"{graph_path}/set_camera"),
            attribute="inputs:cameraPrim",
            target_prim_paths=[self._camera_prim_path],
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
        if (
            not self._publish_labels
            or not camera_type in self._types
            or not camera_type
            in ["semantic_segmentation", "instance_segmentation", "bbox_2d_tight", "bbox_2d_loose", "bbox_3d"]
        ):
            return ""

        return f"{self._namespace}{self._base_topic}/{camera_type}_labels"


class ROS1TF(Graph):
    """The class that implements the ROS1 TF Graph. This class inherits the base class Graph.
    Details: https://docs.omniverse.nvidia.com/isaacsim/latest/ros_tutorials/tutorial_ros_tf.html
    """

    def __init__(
        self,
        prim_path: str = "tf_pub",
        tf_parent_prim_path: str = "/World",
        tf_child_prims_path: List[str] = [],
        topic_name: str = "tf",
        namespace: str = "",
        graph_evaluator: str = "execution",
    ):
        """Initialize the ROS1TF class.
        This class is used to publish the TF transform of the robot or any other object in the Isaac Sim environment.

        Args:
            prim_path (str): The path to the ROS1TF prim. Global path when it starts with `/`, else local to target prim path.
            tf_parent_prim_path (str): The path to the parent prim for the TF transform.
            tf_child_prims_path (List[str]): The paths to the child prims for the TF transform.
            topic_name (str): The name of the ROS1 topic to publish the TF transform.
            namespace (str): The namespace for the ROS1TF.
            graph_evaluator (str): The graph evaluator to use for the ROS1TF.

        Note:
            The target prim path will be indicated by the target_prim argument in the initialize method.

            The `tf_child_prims_path` stores the paths to the child prims for the TF transform. To get the transforms of each linkage on an articulated robot,
            add the robot’s articulation root to the `tf_child_prims_path` arg. All the linkages subsequent to the articulation root will be published automatically.

            The `topic_name` is the name of the ROS1 topic to publish the TF transform. Do not change default value unless you know what you are doing.

            Topic name is `/namespace/topic_name`, default is `/tf`.

        Raises:
            ValueError: If the provided tf_child_prims_path or tf_parent_prim_path is invalid.

        """
        # check ROS1 Bridge Extention
        if get_ros_extension() != "ros":
            carb.log_warn(f'disable ros2 extension: {disable_extension("omni.isaac.ros2_bridge")}')
            carb.log_warn(f'enable ros1 extension: {enable_extension("omni.isaac.ros_bridge")}')

        # Initialize the Super class "object" attribute
        super().__init__(graph_type="ROS1TF")

        # Check if the provided tf_child_prims_path or tf_parent_prim_path is invalid
        if not all([is_prim_path_valid(tf_child_prim_path) for tf_child_prim_path in tf_child_prims_path]):
            carb.log_error(f"Invalid tf_child_prims_path: {tf_child_prims_path}")
            return
        if not is_prim_path_valid(tf_parent_prim_path):
            carb.log_error(f"Invalid tf_parent_prim_path: {tf_parent_prim_path}")
            return

        self._parent_prim_path = tf_parent_prim_path
        self._child_prims_path = tf_child_prims_path
        self._graph_prim_path = prim_path
        self._base_topic_name = topic_name
        self._namespace = f"/{namespace.lstrip('/')}" if namespace != "" else ""
        self._graph_evaluator = graph_evaluator

    def initialize(self, target_prim: _SinglePrimWrapper = None):
        """Method that initializes the graph of the camera.

        Args:
            target_prim (_SinglePrimWrapper): The target Prim to which this graph is attached, which should have `prim_path` attribute.
        """
        # if the `self._graph_prim_path` is not global, and the `target_prim` is not None, then the graph path is set to the target prim path
        if target_prim is not None and self._graph_prim_path[0] != "/":
            self._graph_prim_path = f"{target_prim.prim_path}/{self._graph_prim_path}"

        # Graph configuration
        if self._graph_evaluator == "execution":
            graph_specs = {
                "graph_path": self._graph_prim_path,
                "evaluator_name": "execution",
            }
        elif self._graph_evaluator == "push":
            graph_specs = {
                "graph_path": self._graph_prim_path,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            }
        else:
            carb.log_error(f'Cannot create ROS1 TF graph, graph evaluator type "{self._graph_evaluator}" is not valid')
            return

        # Creating a graph edit configuration to generate ROS tf publisher
        keys = og.Controller.Keys
        graph_config = {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnTick"),
                ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("ros1_publish_tf", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "ros1_publish_tf.inputs:execIn"),
                ("isaac_read_simulation_time.outputs:simulationTime", "ros1_publish_tf.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("ros1_publish_tf.inputs:nodeNamespace", self._namespace),
                ("ros1_publish_tf.inputs:topicName", self._base_topic_name),
                ("ros1_publish_tf.inputs:parentPrim", self._parent_prim_path),
                ("ros1_publish_tf.inputs:targetPrims", self._child_prims_path),
            ],
        }

        # Create the tf graph
        (graph, _, _, _) = og.Controller.edit(graph_specs, graph_config)

        # Run the ROS tf graph once
        og.Controller.evaluate_sync(graph)

        # Also initialize the Super class with updated prim path (only tf graph path)
        super().initialize(self._graph_prim_path)

    def get_tf_topic_name(self) -> str:
        """Method that returns the topic name of the ROS1 TF.

        Returns:
            str: The topic name of the ROS1 TF.
        """
        return f"{self._namespace}/{self._base_topic_name}"


class ROS1Odom(Graph):
    """The class that implements the ROS1 Odom Graph. This class inherits the base class Graph.
    Details: https://docs.omniverse.nvidia.com/isaacsim/latest/ros_tutorials/tutorial_ros_tf.html#
    """

    def __init__(
        self,
        prim_path: str = "odom_pub",
        topic_name: str = "odom",
        namespace: str = "",
        chassis_frame_id: str = "base_link",
        odom_frame_id: str = "odom",
        tf_topic_name: str = "tf",
        tf_namespace: str = "",
        graph_evaluator: str = "execution",
    ):
        """Initialize the ROS1 Odom class.

        Args:
            prim_path (str): Path to the Odom prim. Global path when it starts with `/`, else local to target prim path.
            topic_name (str): Name of the ROS1 topic to publish the odometry messages.
            namespace (str): Namespace for the ROS1 Odom message pub node.
            chassis_frame_id (str): Frame ID of the chassis.
            odom_frame_id (str): Frame ID of the odometry.
            tf_topic_name (str): Name of the ROS1 topic to publish the transform messages.
            tf_namespace (str): Namespace for the ROS1 transform messages pub node.
            graph_evaluator (str): Graph evaluator to be used for the ROS1 Odom class.

        Note:
            The target prim path will be indicated by the target_prim argument in the initialize method.

            This graph will create two message publishers: one for the odometry messages and one for the transform messages.
            The topic name of the odometry message is `/namespace/topic_name`, default is `/odom`.
            The topic name of the transform message is `/tf_topic_name/tf_namespace`, default is `/tf`.
        """
        # check ROS1 Bridge Extention
        if get_ros_extension() != "ros":
            carb.log_warn(f'disable ros2 extension: {disable_extension("omni.isaac.ros2_bridge")}')
            carb.log_warn(f'enable ros1 extension: {enable_extension("omni.isaac.ros_bridge")}')

        # Initialize the Super class "object" attribute
        super().__init__(graph_type="ROS1Odom")

        self._graph_prim_path = prim_path
        self._base_topic_name = topic_name
        self._namespace = f"/{namespace.lstrip('/')}" if namespace != "" else ""
        self._graph_evaluator = graph_evaluator
        self._chassis_frame_id = chassis_frame_id
        self._odom_frame_id = odom_frame_id

        self._tf_topic_name = tf_topic_name
        self._tf_namespace = f"/{tf_namespace.lstrip('/')}" if tf_namespace != "" else ""

    def initialize(self, target_prim: _SinglePrimWrapper = None):
        """Method that initializes the graph of the Odom.

        Args:
            target_prim (_SinglePrimWrapper): The target Prim to which this graph is attached, which should have `prim_path` attribute.
        """
        # if the `self._graph_prim_path` is not global, and the `target_prim` is not None, then the graph path is set to the target prim path
        if target_prim is not None and self._graph_prim_path[0] != "/":
            if not is_prim_path_valid(target_prim.prim_path):
                carb.log_error(f"Invalid target_prim: {target_prim.prim_path}")
                return
            self._graph_prim_path = f"{target_prim.prim_path}/{self._graph_prim_path}"

        # Graph configuration
        if self._graph_evaluator == "execution":
            graph_specs = {
                "graph_path": self._graph_prim_path,
                "evaluator_name": "execution",
            }
        elif self._graph_evaluator == "push":
            graph_specs = {
                "graph_path": self._graph_prim_path,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            }
        else:
            carb.log_error(f'Cannot create ROS1 TF graph, graph evaluator type "{self._graph_evaluator}" is not valid')
            return

        # Creating a graph edit configuration to generate ROS odom publishers
        keys = og.Controller.Keys
        graph_config = {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnTick"),
                ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("isaac_compute_odometry", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                ("ros1_publish_odometry", "omni.isaac.ros_bridge.ROS1PublishOdometry"),
                ("ros1_publish_odometry_tf", "omni.isaac.ros_bridge.ROS1PublishRawTransformTree"),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "isaac_compute_odometry.inputs:execIn"),
                ("on_tick.outputs:tick", "ros1_publish_odometry.inputs:execIn"),
                ("on_tick.outputs:tick", "ros1_publish_odometry_tf.inputs:execIn"),
                ("isaac_read_simulation_time.outputs:simulationTime", "ros1_publish_odometry.inputs:timeStamp"),
                ("isaac_read_simulation_time.outputs:simulationTime", "ros1_publish_odometry_tf.inputs:timeStamp"),
                ("isaac_compute_odometry.outputs:angularVelocity", "ros1_publish_odometry.inputs:angularVelocity"),
                ("isaac_compute_odometry.outputs:linearVelocity", "ros1_publish_odometry.inputs:linearVelocity"),
                ("isaac_compute_odometry.outputs:orientation", "ros1_publish_odometry.inputs:orientation"),
                ("isaac_compute_odometry.outputs:position", "ros1_publish_odometry.inputs:position"),
                ("isaac_compute_odometry.outputs:orientation", "ros1_publish_odometry_tf.inputs:rotation"),
                ("isaac_compute_odometry.outputs:position", "ros1_publish_odometry_tf.inputs:translation"),
            ],
            keys.SET_VALUES: [
                ("ros1_publish_odometry.inputs:nodeNamespace", self._namespace),
                ("ros1_publish_odometry.inputs:topicName", self._base_topic_name),
                ("isaac_compute_odometry.inputs:chassisPrim", target_prim.prim_path),
                ("ros1_publish_odometry.inputs:odomFrameId", self._odom_frame_id),
                ("ros1_publish_odometry.inputs:chassisFrameId", self._chassis_frame_id),
                ("ros1_publish_odometry_tf.inputs:nodeNamespace", self._tf_namespace),
                ("ros1_publish_odometry_tf.inputs:topicName", self._tf_topic_name),
                ("ros1_publish_odometry_tf.inputs:parentFrameId", self._odom_frame_id),
                ("ros1_publish_odometry_tf.inputs:childFrameId", self._chassis_frame_id),
            ],
        }

        # Create the Odom graph
        (graph, _, _, _) = og.Controller.edit(graph_specs, graph_config)

        # Run the ROS Odom graph once
        og.Controller.evaluate_sync(graph)

        # Also initialize the Super class with updated prim path (only Odom graph path)
        super().initialize(self._graph_prim_path)

    def get_topic_name(self) -> str:
        """Method that returns the topic names of the ROS1 Odom.

        Returns:
            str: The topic names of the ROS1 Odom.
        """
        return [f"{self._namespace}/{self._base_topic_name}", f"{self._tf_namespace}/{self._tf_topic_name}"]


class ROS1Clock(Graph):
    """The class that implements the ROS1 Camera Clock. This class inherits the base class Graph.
    Details: https://docs.omniverse.nvidia.com/isaacsim/latest/ros_tutorials/tutorial_ros_clock.html
    """

    def __init__(
        self,
        prim_path: str = "clock_pub",
        topic_name: str = "clock",
        namespace: str = "",
        graph_evaluator: str = "execution",
    ):
        """Initialize the ROS1Clock class.

        Args:
            prim_path (str): The path to the clock publisher prim. If the path starts with `/`, it is a global path; otherwise, it is a local path relative to the target prim path.
            topic_name (str): The name of the clock topic.
            namespace (str): The namespace for the clock pub node.
            graph_evaluator (str): The graph evaluator to be used.

        Note:
            The target prim path will be indicated by the target_prim argument in the initialize method.

            The clock pub topic name is: `/namespace/topic_name`, default is `/clock`.
        """
        # check ROS1 Bridge Extension
        if get_ros_extension() != "ros":
            carb.log_warn(f'disable ros2 extension: {disable_extension("omni.isaac.ros2_bridge")}')
            carb.log_warn(f'enable ros1 extension: {enable_extension("omni.isaac.ros_bridge")}')

        # Initialize the Super class "object" attribute
        super().__init__(graph_type="ROS1Clock")

        self._graph_prim_path = prim_path
        self._base_topic_name = topic_name
        self._namespace = f"/{namespace.lstrip('/')}" if namespace != "" else ""
        self._graph_evaluator = graph_evaluator

    def initialize(self, target_prim: _SinglePrimWrapper = None):
        """Method that initializes the graph of the camera.

        Args:
            target_prim (_SinglePrimWrapper): The target Prim to which this graph is attached, which should have `prim_path` attribute.
        """
        # if the `self._graph_prim_path` is not global, and the `target_prim` is not None, then the graph path is set to the target prim path
        if target_prim is not None and self._graph_prim_path[0] != "/":
            self._graph_prim_path = f"{target_prim.prim_path}/{self._graph_prim_path}"

        # Graph configuration
        if self._graph_evaluator == "execution":
            graph_specs = {
                "graph_path": self._graph_prim_path,
                "evaluator_name": "execution",
            }
        elif self._graph_evaluator == "push":
            graph_specs = {
                "graph_path": self._graph_prim_path,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            }
        else:
            carb.log_error(
                f'Cannot create ROS1 Clock graph, graph evaluator type "{self._graph_evaluator}" is not valid'
            )
            return

        # Creating a graph edit configuration to generate ROS clock publisher
        keys = og.Controller.Keys
        graph_config = {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnTick"),
                ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("ros1_publish_clock", "omni.isaac.ros_bridge.ROS1PublishClock"),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "ros1_publish_clock.inputs:execIn"),
                ("isaac_read_simulation_time.outputs:simulationTime", "ros1_publish_clock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("ros1_publish_clock.inputs:nodeNamespace", self._namespace),
                ("ros1_publish_clock.inputs:topicName", self._base_topic_name),
            ],
        }

        # Create the clock graph
        (graph, _, _, _) = og.Controller.edit(graph_specs, graph_config)

        # Run the ROS Clock graph once
        og.Controller.evaluate_sync(graph)

        # Also initialize the Super class with updated prim path (only clock graph path)
        super().initialize(self._graph_prim_path)

    def get_clock_topic_name(self) -> str:
        """Method that returns the clock topic name.

        Returns:
            str: The clock topic name.
        """
        return f"{self._namespace}/{self._base_topic_name}"

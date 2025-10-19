import omni.graph.core as og
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from pxr import UsdGeom, Gf
from omni.physx.scripts import utils as physx_utils
from pegasus.simulator.ogn.api.shared_node_names import *


def attach_camera_to_drone(drone_prim_path, camera_name, camera_usd, camera_offset, camera_orientation_offset):
    """
    Attach a camera USD to a drone prim with a fixed offset and orientation.

    Args:
        drone_prim_path (str): Path to the drone prim (e.g., "/World/Drone_01").
        camera_name (str): Name of the camera prim to create under the drone.
        camera_usd (str): Path to the camera USD file to reference.
        camera_offset (list[float], optional): Local [x, y, z] offset relative to drone. Default [0,0,0.3].
        camera_orientation_offset (list[float], optional): Quaternion [x, y, z, w] representing additional rotation. Default identity.
    """

    # Construct full camera prim path
    camera_prim_path = f"{drone_prim_path}/{camera_name}"
    prim = get_prim_at_path(camera_prim_path)

    if not prim.IsValid():
        # Create Xform prim under the drone
        prim = define_prim(camera_prim_path, "Xform")
        prim.GetReferences().AddReference(camera_usd)

    # Remove physics if present
    physx_utils.removeRigidBody(prim)
    physx_utils.removeCollider(prim)

    # Get Xformable interface
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()  # reset any previous transforms

    # Apply local translation offset
    xform.AddTranslateOp().Set(Gf.Vec3d(*camera_offset))

    # Corrective rotation for ZED USD orientation (90 deg around Z)
    corrective_quat = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat()
    corrective_rot = Gf.Quatf(
        corrective_quat.GetReal(),
        corrective_quat.GetImaginary()[0],
        corrective_quat.GetImaginary()[1],
        corrective_quat.GetImaginary()[2],
    )

    # User-defined rotation quaternion
    user_rot = Gf.Quatf(
        camera_orientation_offset[3],  # w
        camera_orientation_offset[0],  # x
        camera_orientation_offset[1],  # y
        camera_orientation_offset[2],  # z
    )

    # Combine rotations: corrective first, then user offset
    final_rot = user_rot * corrective_rot
    xform.AddOrientOp().Set(final_rot)

    print(f"Camera '{camera_name}' attached to '{drone_prim_path}' with offset {camera_offset} and rotation {camera_orientation_offset}.")


def add_zed_stereo_camera_subgraph(
    parent_graph_handle: og._omni_graph_core.Graph,
    drone_prim: str,
    px4_node_name: str = "PX4MultirotorNode",
    camera_name: str = "ZEDCamera",
    camera_usd: str = "omniverse://airlab-storage.andrew.cmu.edu:8443/NVIDIA/Assets/Isaac/4.5/Isaac/Sensors/Stereolabs/ZED_X/ZED_X.usd",
    camera_offset: list = [0.12, 0.0, -0.02],
    camera_orientation_offset: list = [0.0, 0.0, 0.0, 1.0],
    robot_name: str = "robot_1",
    stereo_topic_namespace: str = "sensors/front_stereo",
    domain_id: int = 1,
):
    
    controller = og.Controller()

    camera_prim_path = f"{drone_prim}/{camera_name}"

    attach_camera_to_drone(drone_prim, camera_name, camera_usd, camera_offset, camera_orientation_offset)

    parent_graph_path = parent_graph_handle.get_path_to_graph()

    stereo_subgraph_name = f"{camera_name}StereoGraph"
    left_subgraph_name = f"{camera_name}LeftGraph"
    right_subgraph_name = f"{camera_name}RightGraph"

    stereo_subgraph_path = f"{parent_graph_path}/{stereo_subgraph_name}/Subgraph"
    left_subgraph_path = f"{stereo_subgraph_path}/{left_subgraph_name}/Subgraph"
    right_subgraph_path = f"{stereo_subgraph_path}/{right_subgraph_name}/Subgraph"

    left_camera_prim_path = f"{camera_prim_path}/base_link/ZED_X/CameraLeft"
    right_camera_prim_path = f"{camera_prim_path}/base_link/ZED_X/CameraRight"

    # stereo-level nodes
    stereo_playback = f"{camera_name}PlaybackTick"
    stereo_info_helper = f"{camera_name}InfoHelper"
    stereo_ns_const = f"{camera_name}NamespaceConst"
    # stereo_context_forwarder = f"{camera_name}ContextForwarder"
    # stereo_robot_ns_forwarder = f"{camera_name}RobotNamespaceForwarder"
    # stereo_build_ns = f"{camera_name}NamespaceBuilder"
    stereo_context_node = f"{camera_name}ROS2ContextNode"
    robot_name_const = f"{camera_name}RobotNameConst"


    # left child
    left_const_prim = f"{camera_name}LeftConstPrimName"
    # left_build_ns = f"{camera_name}LeftBuildNodeNamespace"
    left_camera_ns_node = f"{camera_name}LeftCameraNamespace"
    left_camera_namespace = f"{robot_name}/{stereo_topic_namespace}/left"
    left_create_rp = f"{camera_name}LeftCreateRenderProduct"
    left_rgb_camera_helper = f"{camera_name}LeftRGBCameraHelper"
    left_depth_camera_helper = f"{camera_name}LeftDepthCameraHelper"
    left_playback = f"{camera_name}LeftPlaybackTick"
    left_context_forwarder = f"{camera_name}LeftContextForwarder"


    # right child
    right_const_prim = f"{camera_name}RightConstPrimName"
    # right_build_ns = f"{camera_name}RightBuildNodeNamespace"
    right_camera_ns_node = f"{camera_name}RightCameraNamespace"
    right_camera_namespace = f"{robot_name}/{stereo_topic_namespace}/right"
    right_create_rp = f"{camera_name}RightCreateRenderProduct"
    right_rgb_camera_helper = f"{camera_name}RightRGBCameraHelper"
    right_depth_camera_helper = f"{camera_name}RightDepthCameraHelper"
    right_playback = f"{camera_name}RightPlaybackTick"
    right_context_forwarder = f"{camera_name}RightContextForwarder"

    controller.edit(
        graph_id=parent_graph_path,
        edit_commands={
            og.Controller.Keys.CREATE_NODES: [
                (
                    stereo_subgraph_name,
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            # Stereo-level
                            (stereo_playback, "omni.graph.action.OnPlaybackTick"),
                            (stereo_info_helper, "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                            (stereo_ns_const, "omni.graph.nodes.ConstantString"),
                            # (stereo_context_forwarder, "omni.graph.nodes.ToUint64"),
                            # (stereo_robot_ns_forwarder, "omni.graph.nodes.ToString"),
                            # (stereo_build_ns, "omni.graph.nodes.BuildString"),
                            (stereo_context_node, "isaacsim.ros2.bridge.ROS2Context"),
                            # (robot_name_const, "omni.graph.nodes.ConstantString"),

                            # Left child subgraph
                            (
                                left_subgraph_name,
                                {
                                    og.Controller.Keys.CREATE_NODES: [
                                        (left_playback, "omni.graph.action.OnPlaybackTick"),
                                        (left_const_prim, "omni.graph.nodes.ConstantString"),
                                        # (left_build_ns, "omni.graph.nodes.BuildString"),
                                        (left_camera_ns_node, "omni.graph.nodes.ConstantString"),
                                        (left_create_rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                                        (left_rgb_camera_helper, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                                        (left_depth_camera_helper, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                                        (left_context_forwarder, "omni.graph.nodes.ToUint64"),
                                    ],
                                    og.Controller.Keys.PROMOTE_ATTRIBUTES: [
                                        (f"{left_const_prim}.inputs:value", "outputs:frameId"),
                                        (f"{left_create_rp}.outputs:renderProductPath", "outputs:renderProductPath"),
                                        # (f"{left_rgb_camera_helper}.inputs:context", "inputs:rgbContext"),
                                        # (f"{left_depth_camera_helper}.inputs:context", "inputs:depthContext"),
                                        (f"{left_context_forwarder}.inputs:value", "inputs:context"),
                                        # (f"{left_build_ns}.inputs:a", "inputs:robotName"),
                                        # (f"{left_build_ns}.inputs:c", "inputs:stereoNamespace"),
                                    ],
                                    og.Controller.Keys.CONNECT: [
                                        # Execution wiring
                                        (f"{left_playback}.outputs:tick", f"{left_create_rp}.inputs:execIn"),

                                        # Left RGB camera connections
                                        (f"{left_context_forwarder}.outputs:converted", f"{left_rgb_camera_helper}.inputs:context"),
                                        # (f"{left_build_ns}.outputs:value", f"{left_rgb_camera_helper}.inputs:nodeNamespace"),
                                        (f"{left_camera_ns_node}.inputs:value", f"{left_rgb_camera_helper}.inputs:nodeNamespace"),
                                        (f"{left_const_prim}.inputs:value", f"{left_rgb_camera_helper}.inputs:frameId"),
                                        (f"{left_create_rp}.outputs:renderProductPath", f"{left_rgb_camera_helper}.inputs:renderProductPath"),
                                        (f"{left_create_rp}.outputs:execOut", f"{left_rgb_camera_helper}.inputs:execIn"),

                                        # Left Depth camera connections
                                        (f"{left_context_forwarder}.outputs:converted", f"{left_depth_camera_helper}.inputs:context"),
                                        # (f"{left_build_ns}.outputs:value", f"{left_depth_camera_helper}.inputs:nodeNamespace"),
                                        (f"{left_camera_ns_node}.inputs:value", f"{left_depth_camera_helper}.inputs:nodeNamespace"),
                                        (f"{left_const_prim}.inputs:value", f"{left_depth_camera_helper}.inputs:frameId"),
                                        (f"{left_create_rp}.outputs:renderProductPath", f"{left_depth_camera_helper}.inputs:renderProductPath"),
                                        (f"{left_create_rp}.outputs:execOut", f"{left_depth_camera_helper}.inputs:execIn"),
                                    ],
                                    og.Controller.Keys.SET_VALUES: [ # TODO Make rgb and depth optional and also have control over topic names
                                        (("inputs:topicName", left_rgb_camera_helper), "image_rect"),
                                        (("inputs:type", left_rgb_camera_helper), "rgb"),
                                        (("inputs:topicName", left_depth_camera_helper), "depth_ground_truth"),
                                        (("inputs:type", left_depth_camera_helper), "depth"),
                                        (("inputs:cameraPrim", left_create_rp), left_camera_prim_path),
                                        (("inputs:height", left_create_rp), 300),
                                        (("inputs:width", left_create_rp), 480),
                                        # (("inputs:b", left_build_ns), "/"),
                                        # (("inputs:d", left_build_ns), "/left"),
                                        (("inputs:value", left_camera_ns_node), left_camera_namespace),
                                    ],
                                },
                            ),

                            # Right child subgraph
                            (
                                right_subgraph_name,
                                {
                                    og.Controller.Keys.CREATE_NODES: [
                                        (right_playback, "omni.graph.action.OnPlaybackTick"),
                                        (right_const_prim, "omni.graph.nodes.ConstantString"),
                                        (right_camera_ns_node, "omni.graph.nodes.ConstantString"),
                                        # (right_build_ns, "omni.graph.nodes.BuildString"),
                                        (right_create_rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                                        (right_rgb_camera_helper, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                                        (right_depth_camera_helper, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                                        (right_context_forwarder, "omni.graph.nodes.ToUint64"),
                                    ],
                                    og.Controller.Keys.PROMOTE_ATTRIBUTES: [
                                        (f"{right_const_prim}.inputs:value", "outputs:frameId"),
                                        (f"{right_create_rp}.outputs:renderProductPath", "outputs:renderProductPath"),
                                        # (f"{right_rgb_camera_helper}.inputs:context", "inputs:rgbContext"),
                                        # (f"{right_depth_camera_helper}.inputs:context", "inputs:depthContext"),
                                        (f"{right_context_forwarder}.inputs:value", f"inputs:context"),
                                        # (f"{right_build_ns}.inputs:a", "inputs:robotName"),
                                        # (f"{right_build_ns}.inputs:c", "inputs:stereoNamespace"),
                                    ],
                                    og.Controller.Keys.CONNECT: [
                                        # Execution wiring
                                        (f"{right_playback}.outputs:tick", f"{right_create_rp}.inputs:execIn"),

                                        # Right RGB camera connections
                                        (f"{right_context_forwarder}.outputs:converted", f"{right_rgb_camera_helper}.inputs:context"),
                                        # (f"{right_build_ns}.outputs:value", f"{right_rgb_camera_helper}.inputs:nodeNamespace"),
                                        (f"{right_camera_ns_node}.inputs:value", f"{right_rgb_camera_helper}.inputs:nodeNamespace"),
                                        (f"{right_const_prim}.inputs:value", f"{right_rgb_camera_helper}.inputs:frameId"),
                                        (f"{right_create_rp}.outputs:renderProductPath", f"{right_rgb_camera_helper}.inputs:renderProductPath"),
                                        (f"{right_create_rp}.outputs:execOut", f"{right_rgb_camera_helper}.inputs:execIn"),

                                        # Right Depth camera connections
                                        (f"{right_context_forwarder}.outputs:converted", f"{right_depth_camera_helper}.inputs:context"),
                                        # (f"{right_build_ns}.outputs:value", f"{right_depth_camera_helper}.inputs:nodeNamespace"),
                                        (f"{right_camera_ns_node}.inputs:value", f"{right_depth_camera_helper}.inputs:nodeNamespace"),
                                        (f"{right_const_prim}.inputs:value", f"{right_depth_camera_helper}.inputs:frameId"),
                                        (f"{right_create_rp}.outputs:renderProductPath", f"{right_depth_camera_helper}.inputs:renderProductPath"),
                                        (f"{right_create_rp}.outputs:execOut", f"{right_depth_camera_helper}.inputs:execIn"),
                                    ],
                                    og.Controller.Keys.SET_VALUES: [ # TODO Make rgb and depth optional and also have control over topic names
                                        (("inputs:topicName", right_rgb_camera_helper), "image_rect"),
                                        (("inputs:type", right_rgb_camera_helper), "rgb"),
                                        (("inputs:topicName", right_depth_camera_helper), "depth_ground_truth"),
                                        (("inputs:type", right_depth_camera_helper), "depth"),
                                        (("inputs:cameraPrim", right_create_rp), right_camera_prim_path),
                                        (("inputs:height", right_create_rp), 300),
                                        (("inputs:width", right_create_rp), 480),
                                        # (("inputs:b", right_build_ns), "/"),
                                        # (("inputs:d", right_build_ns), "/right"),
                                        (("inputs:value", right_camera_ns_node), right_camera_namespace),
                                    ],
                                },
                            ),
                        ],

                        # promote stereo-level attributes (connect promoted inputs to helper)
                        og.Controller.Keys.PROMOTE_ATTRIBUTES: [
                            # (f"{stereo_context_forwarder}.inputs:value", "inputs:context"),
                            # (f"{stereo_robot_ns_forwarder}.inputs:value", "inputs:robotNamespace"),
                        ],

                        og.Controller.Keys.CONNECT: [
                            # (f"{robot_name_const}.inputs:value", f"{stereo_build_ns}.inputs:a"),
                            # (f"{stereo_ns_const}.inputs:value", f"{stereo_build_ns}.inputs:b"),

                            # (f"{robot_name_const}.inputs:value", f"{left_subgraph_name}.inputs:robotName"),
                            # (f"{robot_name_const}.inputs:value", f"{right_subgraph_name}.inputs:robotName"),

                            (f"{stereo_context_node}.outputs:context", f"{left_subgraph_name}.inputs:context"),
                            (f"{stereo_context_node}.outputs:context", f"{right_subgraph_name}.inputs:context"),
                            (f"{stereo_context_node}.outputs:context", f"{stereo_info_helper}.inputs:context"),

                            (f"{left_subgraph_name}.outputs:frameId", f"{stereo_info_helper}.inputs:frameId"),
                            (f"{left_subgraph_name}.outputs:renderProductPath", f"{stereo_info_helper}.inputs:renderProductPath"),

                            (f"{right_subgraph_name}.outputs:frameId", f"{stereo_info_helper}.inputs:frameIdRight"),
                            (f"{right_subgraph_name}.outputs:renderProductPath", f"{stereo_info_helper}.inputs:renderProductPathRight"),
                            
                            # (f"{stereo_build_ns}.outputs:value", f"{stereo_info_helper}.inputs:nodeNamespace"),
                            # (f"{stereo_ns_const}.inputs:value", f"{left_subgraph_name}.inputs:stereoNamespace"),
                            # (f"{stereo_ns_const}.inputs:value", f"{right_subgraph_name}.inputs:stereoNamespace"),

                            (f"{stereo_playback}.outputs:tick", f"{stereo_info_helper}.inputs:execIn"),

                            # (f"{stereo_context_node}.outputs:converted", f"{stereo_info_helper}.inputs:context"),
                        ],

                        og.Controller.Keys.SET_VALUES: [
                            # (("inputs:value", stereo_ns_const), camera_name), # TODO: Change this to be specified by input
                            (f"{stereo_info_helper}.inputs:topicName", f"left/camera_info"),
                            (f"{stereo_info_helper}.inputs:topicNameRight", f"right/camera_info"),
                            # (("inputs:value", robot_name_const), robot_name),
                            (f"{stereo_context_node}.inputs:domain_id", domain_id),
                            (f"{stereo_info_helper}.inputs:nodeNamespace", f"{robot_name}/{stereo_topic_namespace}"),
                        ],
                    },
                ),
            ],
        },
    )

    # controller.edit(
    #     graph_id=left_subgraph_path,
    #     edit_commands={
    #         og.Controller.Keys.CONNECT: [
    #             (f"{left_const_prim}.inputs:value", f"{left_rgb_camera_helper}.inputs:frameId"),
    #             (f"{left_const_prim}.inputs:value", f"{left_depth_camera_helper}.inputs:frameId"),
    #         ]
    #     },
    # )

    # set constants
    controller.edit(
        graph_id=parent_graph_path,
        edit_commands={
            # og.Controller.Keys.SET_VALUES: [
            #     (f"{stereo_subgraph_name}.{stereo_ns_const}.inputs:value", camera_name),
            #     (f"{stereo_subgraph_name}.{left_const_prim}.inputs:value", f"{camera_name}_left"),
            #     (f"{stereo_subgraph_name}.{right_const_prim}.inputs:value", f"{camera_name}_right"),
            # ]
        },
    )

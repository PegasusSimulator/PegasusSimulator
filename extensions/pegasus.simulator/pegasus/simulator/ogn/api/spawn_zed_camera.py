import omni.graph.core as og
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from pxr import UsdGeom, Gf, Sdf, UsdUtils
from omni.physx.scripts import utils as physx_utils
import omni


def attach_camera_to_drone(drone_prim_path, camera_name, camera_usd, camera_offset, camera_orientation_offset, left_frame_id, right_frame_id):
    """
    Attach a camera USD to a drone prim with a fixed offset and orientation.

    Args:
        drone_prim_path (str): Path to the drone prim (e.g., "/World/Drone_01").
        camera_name (str): Name of the camera prim to create under the drone.
        camera_usd (str): Path to the camera USD file to reference.
        camera_offset (list[float], optional): Local [x, y, z] offset relative to drone. Default [0,0,0.3].
        camera_orientation_offset (list[float], optional): Quaternion [x, y, z, w] representing additional rotation. Default identity.
    """

    stage = omni.usd.get_context().get_stage()

    camera_prim_path = f"{drone_prim_path}/{camera_name}"
    prim = get_prim_at_path(camera_prim_path)

    if not prim.IsValid():
        prim = define_prim(camera_prim_path, "Xform")
        prim.GetReferences().AddReference(camera_usd)
        # UsdUtils.FlattenLayerStack(stage.GetEditTarget().GetLayer(), Sdf.Path(camera_prim_path))

    physx_utils.removeCollider(prim)

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()

    xform_ops_to_set = []

    corrective_quat = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat()
    corrective_rot = Gf.Quatf(
        corrective_quat.GetReal(),
        corrective_quat.GetImaginary()[0],
        corrective_quat.GetImaginary()[1],
        corrective_quat.GetImaginary()[2],
    )

    user_rot = Gf.Quatf(
        camera_orientation_offset[3],
        camera_orientation_offset[0],
        camera_orientation_offset[1],
        camera_orientation_offset[2],
    )

    final_rot = user_rot * corrective_rot

    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(*camera_offset))
    xform_ops_to_set.append(translate_op)

    orient_op = xform.AddOrientOp()
    orient_op.Set(final_rot)
    xform_ops_to_set.append(orient_op)

    xform.SetXformOpOrder(xform_ops_to_set)

    omni.physx.scripts.utils.createJoint(
        stage,
        joint_type="Fixed",
        from_prim=stage.GetPrimAtPath(f"{drone_prim_path}/body/body"),
        to_prim=stage.GetPrimAtPath(camera_prim_path),
    )

    # Rename the internal sensor prim
    left_sensor_prim_path_old = Sdf.Path(f"{camera_prim_path}/base_link/ZED_X/CameraLeft") # Original name
    right_sensor_prim_path_old = Sdf.Path(f"{camera_prim_path}/base_link/ZED_X/CameraRight") # Original name
    left_sensor_prim_path_new = Sdf.Path(f"{camera_prim_path}/base_link/ZED_X/{left_frame_id}")  # Desired new name
    right_sensor_prim_path_new = Sdf.Path(f"{camera_prim_path}/base_link/ZED_X/{right_frame_id}")  # Desired new name

    omni.kit.app.get_app().update()

    omni.kit.commands.execute(
        "CopyPrim",
        path_from=left_sensor_prim_path_old,
        path_to=left_sensor_prim_path_new,
    )
    omni.kit.commands.execute(
        "CopyPrim",
        path_from=right_sensor_prim_path_old,
        path_to=right_sensor_prim_path_new,
    )

    omni.kit.app.get_app().update() # Ensure deletion is processed

    print(f"Camera '{camera_name}' attached to '{drone_prim_path}'.")


def add_zed_stereo_camera_subgraph(
    parent_graph_handle: og._omni_graph_core.Graph,
    drone_prim: str,
    px4_node_name: str = "PX4MultirotorNode",
    camera_name: str = "ZEDCamera",
    camera_usd: str = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Sensors/Stereolabs/ZED_X/ZED_X.usd",
    camera_offset: list = [0.12, 0.0, -0.02],
    camera_orientation_offset: list = [0.0, 0.0, 0.0, 1.0],
    robot_name: str = "robot_1",
    stereo_topic_namespace: str = "front_stereo",
    sensors_topic_namespace: str = "sensors",
    domain_id: int = 1,
    stereo_frame_id: str = "base_link",
    left_frame_id: str = "camera_left",
    right_frame_id: str = "camera_right",
):
    controller = og.Controller()

    camera_prim_path = f"{drone_prim}/{camera_name}"

    attach_camera_to_drone(drone_prim, camera_name, camera_usd, camera_offset, camera_orientation_offset, left_frame_id, right_frame_id)

    parent_graph_path = parent_graph_handle.get_path_to_graph()
    stereo_graph_name = f"{robot_name}_{camera_name}StereoGraph"

    # Prim paths
    left_camera_prim_path = f"{camera_prim_path}/base_link/ZED_X/CameraLeft"
    right_camera_prim_path = f"{camera_prim_path}/base_link/ZED_X/CameraRight"

    # Topic namespaces
    left_ns = f"{robot_name}/{sensors_topic_namespace}/{stereo_topic_namespace}/left"
    right_ns = f"{robot_name}/{sensors_topic_namespace}/{stereo_topic_namespace}/right"
    stereo_ns = f"{robot_name}/{sensors_topic_namespace}/{stereo_topic_namespace}"

    # Node names
    domain_id_const = f"{robot_name}_{camera_name}_DomainIdConst"
    context_forwarder = f"{robot_name}_{camera_name}ContextForwarder"
    stereo_playback = f"{robot_name}_{camera_name}_PlaybackTick"
    stereo_context_node = f"{robot_name}_{camera_name}_ROS2ContextNode"
    stereo_info_helper = f"{robot_name}_{camera_name}_StereoInfoHelper"
    ros2Context_node_name = f"{robot_name}_ROS2Context"

    # Constant node names
    left_frame_const = f"{robot_name}_{camera_name}_LeftFrameIdConst"
    right_frame_const = f"{robot_name}_{camera_name}_RightFrameIdConst"
    # stereo_frame_const = f"{robot_name}_{camera_name}_StereoFrameIdConst"
    left_ns_const = f"{robot_name}_{camera_name}_LeftNsConst"
    right_ns_const = f"{robot_name}_{camera_name}_RightNsConst"
    stereo_ns_const = f"{robot_name}_{camera_name}_StereoNsConst"

    # Left camera nodes
    left_create_rp = f"{robot_name}_{camera_name}_LeftCreateRenderProduct"
    left_rgb_camera_helper = f"{robot_name}_{camera_name}_LeftRGBCameraHelper"
    left_depth_camera_helper = f"{robot_name}_{camera_name}_LeftDepthCameraHelper"

    # Right camera nodes
    right_create_rp = f"{robot_name}_{camera_name}_RightCreateRenderProduct"
    right_rgb_camera_helper = f"{robot_name}_{camera_name}_RightRGBCameraHelper"
    right_depth_camera_helper = f"{robot_name}_{camera_name}_RightDepthCameraHelper"

    # Create the flat subgraph
    controller.edit(
        graph_id=parent_graph_path,
        edit_commands={
            og.Controller.Keys.CREATE_NODES: [
                (
                    stereo_graph_name,
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            # Core nodes
                            (stereo_playback, "omni.graph.action.OnPlaybackTick"),
                            (stereo_context_node, "isaacsim.ros2.bridge.ROS2Context"),
                            (stereo_info_helper, "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                            (domain_id_const, "omni.graph.nodes.ConstantUChar"),
                            (context_forwarder, "omni.graph.nodes.ToUint64"),
                            # Constant string nodes
                            (left_frame_const, "omni.graph.nodes.ConstantString"),
                            (right_frame_const, "omni.graph.nodes.ConstantString"),
                            # (stereo_frame_const, "omni.graph.nodes.ConstantString"),
                            (left_ns_const, "omni.graph.nodes.ConstantString"),
                            (right_ns_const, "omni.graph.nodes.ConstantString"),
                            (stereo_ns_const, "omni.graph.nodes.ConstantString"),
                            # Left nodes
                            (left_create_rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                            (left_rgb_camera_helper, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                            (left_depth_camera_helper, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                            # Right nodes
                            (right_create_rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                            (right_rgb_camera_helper, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                            (right_depth_camera_helper, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ],
                        og.Controller.Keys.PROMOTE_ATTRIBUTES: [
                            (f"{context_forwarder}.inputs:value", "inputs:context"),
                        ],
                        og.Controller.Keys.CONNECT: [
                            # --- Stereo-level connections ---
                            # Playback tick
                            (f"{stereo_playback}.outputs:tick", f"{left_create_rp}.inputs:execIn"),
                            (f"{stereo_playback}.outputs:tick", f"{right_create_rp}.inputs:execIn"),
                            (f"{stereo_playback}.outputs:tick", f"{stereo_info_helper}.inputs:execIn"),

                            # Context wiring
                            # (f"{domain_id_const}.inputs:value", f"{stereo_context_node}.inputs:domain_id"),
                            # (f"{stereo_context_node}.outputs:context", f"{stereo_info_helper}.inputs:context"),
                            (f"{context_forwarder}.outputs:converted", f"{stereo_info_helper}.inputs:context"),
                             
                            # Stereo info helper render paths
                            (f"{left_create_rp}.outputs:renderProductPath", f"{stereo_info_helper}.inputs:renderProductPath"),
                            (f"{right_create_rp}.outputs:renderProductPath", f"{stereo_info_helper}.inputs:renderProductPathRight"),

                            # Connect constant strings to frame IDs and namespaces
                            (f"{left_frame_const}.inputs:value", f"{stereo_info_helper}.inputs:frameId"),
                            (f"{right_frame_const}.inputs:value", f"{stereo_info_helper}.inputs:frameIdRight"),
                            (f"{stereo_ns_const}.inputs:value", f"{stereo_info_helper}.inputs:nodeNamespace"),
                            
                            # # --- Monocular Camera-level connections ---

                            # # --- --- Left camera connections --- ---
                            # # Left RGB camera helpers
                            # (f"{stereo_context_node}.outputs:context", f"{left_rgb_camera_helper}.inputs:context"),
                            (f"{context_forwarder}.outputs:converted", f"{left_rgb_camera_helper}.inputs:context"),
                            (f"{left_create_rp}.outputs:renderProductPath", f"{left_rgb_camera_helper}.inputs:renderProductPath"),
                            (f"{left_create_rp}.outputs:execOut", f"{left_rgb_camera_helper}.inputs:execIn"),
                            (f"{left_frame_const}.inputs:value", f"{left_rgb_camera_helper}.inputs:frameId"),
                            (f"{left_ns_const}.inputs:value", f"{left_rgb_camera_helper}.inputs:nodeNamespace"),

                            # Left Depth camera helpers
                            # (f"{stereo_context_node}.outputs:context", f"{left_depth_camera_helper}.inputs:context"),
                            (f"{context_forwarder}.outputs:converted", f"{left_depth_camera_helper}.inputs:context"),
                            (f"{left_create_rp}.outputs:renderProductPath", f"{left_depth_camera_helper}.inputs:renderProductPath"),
                            (f"{left_create_rp}.outputs:execOut", f"{left_depth_camera_helper}.inputs:execIn"),
                            (f"{left_frame_const}.inputs:value", f"{left_depth_camera_helper}.inputs:frameId"),
                            (f"{left_ns_const}.inputs:value", f"{left_depth_camera_helper}.inputs:nodeNamespace"),

                            # --- --- Right camera connections --- ---
                            # Right RGB camera helpers
                            # (f"{stereo_context_node}.outputs:context", f"{right_rgb_camera_helper}.inputs:context"),
                            (f"{context_forwarder}.outputs:converted", f"{right_rgb_camera_helper}.inputs:context"),
                            (f"{right_create_rp}.outputs:renderProductPath", f"{right_rgb_camera_helper}.inputs:renderProductPath"),
                            (f"{right_create_rp}.outputs:execOut", f"{right_rgb_camera_helper}.inputs:execIn"),
                            (f"{right_frame_const}.inputs:value", f"{right_rgb_camera_helper}.inputs:frameId"),
                            (f"{right_ns_const}.inputs:value", f"{right_rgb_camera_helper}.inputs:nodeNamespace"),

                            # Right Depth camera helpers
                            # (f"{stereo_context_node}.outputs:context", f"{right_depth_camera_helper}.inputs:context"),
                            (f"{context_forwarder}.outputs:converted", f"{right_depth_camera_helper}.inputs:context"),
                            (f"{right_create_rp}.outputs:renderProductPath", f"{right_depth_camera_helper}.inputs:renderProductPath"),
                            (f"{right_create_rp}.outputs:execOut", f"{right_depth_camera_helper}.inputs:execIn"),
                            (f"{right_frame_const}.inputs:value", f"{right_depth_camera_helper}.inputs:frameId"),
                            (f"{right_ns_const}.inputs:value", f"{right_depth_camera_helper}.inputs:nodeNamespace"),
                        ],

                        og.Controller.Keys.SET_VALUES: [
                            # Constants
                            (("inputs:value", left_frame_const), left_frame_id),
                            (("inputs:value", right_frame_const), right_frame_id),
                            (("inputs:value", left_ns_const), left_ns),
                            (("inputs:value", right_ns_const), right_ns),
                            (("inputs:value", stereo_ns_const), stereo_ns),
                            # (("inputs:value", domain_id_const), domain_id), # Context
                            # Stereo helper topics
                            (("inputs:topicName", stereo_info_helper), "left/camera_info"),
                            (("inputs:topicNameRight", stereo_info_helper), "right/camera_info"),
                            # Left render product
                            (("inputs:cameraPrim", left_create_rp), left_camera_prim_path),
                            (("inputs:height", left_create_rp), 300),
                            (("inputs:width", left_create_rp), 480),
                            # Left helpers
                            (("inputs:type", left_rgb_camera_helper), "rgb"),
                            (("inputs:type", left_depth_camera_helper), "depth"),
                            (("inputs:topicName", left_rgb_camera_helper), "image_rect"),
                            (("inputs:topicName", left_depth_camera_helper), "depth_ground_truth"),
                            # Right render product
                            (("inputs:cameraPrim", right_create_rp), right_camera_prim_path),
                            (("inputs:height", right_create_rp), 300),
                            (("inputs:width", right_create_rp), 480),
                            # Right helpers
                            (("inputs:type", right_rgb_camera_helper), "rgb"),
                            (("inputs:type", right_depth_camera_helper), "depth"),
                            (("inputs:topicName", right_rgb_camera_helper), "image_rect"),
                            (("inputs:topicName", right_depth_camera_helper), "depth_ground_truth"),
                        ],
                    },
                )
            ],
            og.Controller.Keys.CONNECT: [
                # (f"{ros2Context_node_name}.outputs:context", f"{stereo_graph_name}.inputs:context"),

            ],
        },
    )

    print(f"Flat stereo camera graph with constant structure created for '{camera_name}' under '{drone_prim}'.")

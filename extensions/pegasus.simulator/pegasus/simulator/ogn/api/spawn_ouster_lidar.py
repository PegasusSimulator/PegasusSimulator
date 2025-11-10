import omni.graph.core as og
from isaacsim.core.utils.prims import define_prim
from pxr import UsdGeom, Gf, UsdPhysics, Sdf
from omni.physx.scripts import utils as physx_utils
import omni
import carb

# Default Ouster OS1 USD asset
OUSTER_LIDAR_USD_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/4.5/Isaac/Sensors/Ouster/OS1/OS1_REV6_128_10hz___512_resolution.usd"
)


def attach_lidar_to_drone(
    drone_prim_path: str,
    lidar_name: str,
    lidar_usd: str,
    lidar_offset: list[float],
    lidar_rotation_offset: list[float],
    frame_id: str,
) -> str:
    """
    Attach a LiDAR USD to a drone prim (ZED-style), converting RPY -> quaternion and
    applying a corrective rotation to align with USD Z-forward convention.

    Args:
        drone_prim_path: Path to the drone prim (e.g. "/World/Drone_01").
        lidar_name: Name for the LiDAR prim under the drone.
        lidar_usd: URL or path to the LiDAR USD asset.
        lidar_offset: [x, y, z] translation offset relative to the drone.
        lidar_rotation_rpy: [roll, pitch, yaw] in degrees.
        frame_id: Name for the internal sensor prim (renamed from "sensor").

    Returns:
        The path to the renamed sensor prim (str) or None on failure.
    """

    stage = omni.usd.get_context().get_stage()
    lidar_prim_path = f"{drone_prim_path}/{lidar_name}"

    # Remove existing LiDAR prim
    existing = stage.GetPrimAtPath(lidar_prim_path)
    if existing.IsValid():
        carb.log_info(f"Deleting existing LiDAR prim at {lidar_prim_path}")
        omni.kit.commands.execute("DeletePrim", path=lidar_prim_path)

    # Create new LiDAR prim and reference USD
    carb.log_info(f"Creating LiDAR prim '{lidar_prim_path}' referencing '{lidar_usd}'")
    lidar_prim = define_prim(lidar_prim_path, "Xform")
    lidar_prim.GetReferences().AddReference(lidar_usd)

    # Compute rotation
    roll_deg, pitch_deg, yaw_deg = lidar_rotation_offset
    roll_rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), roll_deg)
    pitch_rot = Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch_deg)
    yaw_rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg)
    combined_rot = yaw_rot * pitch_rot * roll_rot
    user_quat = combined_rot.GetQuat()

    # Corrective rotation to align LiDAR’s local axes (Z-forward)
    corrective_quat = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat()

    user_rot = Gf.Quatf(user_quat.GetReal(), *user_quat.GetImaginary())
    corrective_rot = Gf.Quatf(corrective_quat.GetReal(), *corrective_quat.GetImaginary())
    final_rot = user_rot * corrective_rot

    # Apply translation and orientation
    xform = UsdGeom.Xformable(lidar_prim)
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp()
    orient_op = xform.AddOrientOp()
    translate_op.Set(Gf.Vec3d(*lidar_offset))
    orient_op.Set(final_rot)
    xform.SetXformOpOrder([translate_op, orient_op])

    # Rename internal sensor prim (done via copy + delete to preserve references)
    sensor_old_path = f"{lidar_prim_path}/sensor"
    sensor_new_path = f"{lidar_prim_path}/{frame_id}"
    sensor_old = stage.GetPrimAtPath(sensor_old_path)

    if sensor_old.IsValid():
        carb.log_info(f"Renaming internal LiDAR sensor '{sensor_old_path}' → '{sensor_new_path}'")
        omni.kit.commands.execute(
            "CopyPrim",
            path_from=sensor_old_path,
            path_to=sensor_new_path,
            duplicate_layers=True,
        )
        omni.kit.commands.execute("DeletePrim", path=sensor_old_path)
    else:
        carb.log_warn(f"No '/sensor' child found under '{lidar_prim_path}'.")

    # Apply RigidBody API and remove collider
    try:
        if not UsdPhysics.RigidBodyAPI(lidar_prim):
            UsdPhysics.RigidBodyAPI.Apply(lidar_prim)
    except Exception:
        carb.log_warn(f"RigidBodyAPI could not be applied to '{lidar_prim_path}'.")

    try:
        physx_utils.removeCollider(lidar_prim)
    except Exception:
        carb.log_warn(f"Failed to remove collider from '{lidar_prim_path}'.")

    # Fixed joint to drone body
    drone_body_path = f"{drone_prim_path}/body/body"
    drone_body_prim = stage.GetPrimAtPath(drone_body_path)
    if not drone_body_prim.IsValid():
        carb.log_error(f"Drone body prim '{drone_body_path}' not found; cannot attach LiDAR.")
        return None

    joint = physx_utils.createJoint(
        stage,
        joint_type="Fixed",
        from_prim=drone_body_prim,
        to_prim=lidar_prim,
    )

    if not joint or not joint.IsValid():
        carb.log_error(f"Failed to create fixed joint for LiDAR '{lidar_prim_path}'.")
        return None

    carb.log_info(f"LiDAR '{lidar_name}' successfully attached to '{drone_prim_path}' as '{sensor_new_path}'.")
    return sensor_new_path


def add_ouster_lidar_subgraph(
    parent_graph_handle: og._omni_graph_core.Graph,
    drone_prim: str,
    lidar_name: str = "OS1_REV6_128_10hz___512_resolution",
    lidar_topic_name: str = "point_cloud",
    lidar_usd: str = OUSTER_LIDAR_USD_URL,
    lidar_offset: list[float] = [0.0, 0.0, 0.025],
    lidar_rotation_offset: list[float] = [0.0, 0.0, 0.0],
    lidar_topic_namespace: str = "sensors/lidar",
    lidar_frame_id: str = "lidar",
    frame_height: int = 720,
    frame_width: int = 1280,
    robot_name: str = "robot_1",
):
    """
    Adds a lidar and builds a minimal ROS2 OmniGraph subgraph that publishes the LiDAR's point cloud.

    The graph includes:
        - Playback tick trigger
        - One simulation step node
        - LiDAR render product creation
        - RTX LiDAR ROS2 helper
        - Frame and namespace constant nodes

    Args:
        parent_graph_handle (og.Graph): The parent OmniGraph handle.
        drone_prim (str): Path to the drone prim.
        lidar_name (str): Name of the LiDAR prim.
        lidar_topic_name (str): ROS topic name (e.g. "point_cloud").
        lidar_usd (str): Path or URL to LiDAR USD asset.
        lidar_offset (list[float]): [x, y, z] offset relative to drone.
        lidar_rotation_offset (list[float]): [roll, pitch, yaw] in degrees.
        lidar_topic_namespace (str): ROS topic namespace.
        lidar_frame_id (str): Frame ID for ROS messages.
        frame_height (int): Render product height.
        frame_width (int): Render product width.
        robot_name (str): Robot name prefix for ROS topic namespace.

    Returns:
        None
    """
    controller = og.Controller()

    lidar_sensor_prim = attach_lidar_to_drone(
        drone_prim_path=drone_prim,
        lidar_name=lidar_name,
        lidar_usd=lidar_usd,
        lidar_offset=lidar_offset,
        lidar_rotation_offset=lidar_rotation_offset,
        frame_id=lidar_frame_id,
    )

    if lidar_sensor_prim is None:
        carb.log_error("LiDAR attachment failed; aborting subgraph creation.")
        return

    parent_graph_path = parent_graph_handle.get_path_to_graph()
    lidar_subgraph_name = f"{lidar_name}Graph"

    playback_tick = f"{lidar_name}PlaybackTick"
    create_render = f"{lidar_name}CreateRenderProduct"
    rtx_helper = f"{lidar_name}ROS2RtxLidarHelper"
    run_one_sim_frame = f"{lidar_name}RunOneSimFrame"
    frame_const = f"{lidar_name}FrameIdConst"
    ns_const = f"{lidar_name}NamespaceConst"

    controller.edit(
        graph_id=parent_graph_path,
        edit_commands={
            og.Controller.Keys.CREATE_NODES: [
                (
                    lidar_subgraph_name,
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            (playback_tick, "omni.graph.action.OnPlaybackTick"),
                            (create_render, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                            (rtx_helper, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                            (frame_const, "omni.graph.nodes.ConstantString"),
                            (ns_const, "omni.graph.nodes.ConstantString"),
                            (run_one_sim_frame, "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                        ],
                        og.Controller.Keys.PROMOTE_ATTRIBUTES: [
                            (f"{rtx_helper}.inputs:context", f"inputs:context"),
                        ],
                        og.Controller.Keys.SET_VALUES: [
                            (("inputs:value", frame_const), lidar_frame_id),
                            (("inputs:value", ns_const), f"{robot_name}/{lidar_topic_namespace}"),
                            (("inputs:cameraPrim", create_render), lidar_sensor_prim),
                            (("inputs:height", create_render), frame_height),
                            (("inputs:width", create_render), frame_width),
                            (("inputs:topicName", rtx_helper), lidar_topic_name),
                            (("inputs:type", rtx_helper), "point_cloud"),
                        ],
                        og.Controller.Keys.CONNECT: [
                            (f"{playback_tick}.outputs:tick", f"{run_one_sim_frame}.inputs:execIn"),
                            (f"{run_one_sim_frame}.outputs:step", f"{create_render}.inputs:execIn"),
                            (f"{create_render}.outputs:execOut", f"{rtx_helper}.inputs:execIn"),
                            (f"{create_render}.outputs:renderProductPath", f"{rtx_helper}.inputs:renderProductPath"),
                            (f"{frame_const}.inputs:value", f"{rtx_helper}.inputs:frameId"),
                            (f"{ns_const}.inputs:value", f"{rtx_helper}.inputs:nodeNamespace"),
                        ],
                    },
                )
            ]
        },
    )

    carb.log_info(f"LiDAR subgraph '{lidar_subgraph_name}' added under '{parent_graph_path}'.")
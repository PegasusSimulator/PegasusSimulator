import omni.graph.core as og
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from pxr import UsdGeom, Gf, UsdPhysics, Sdf
from omni.physx.scripts import utils as physx_utils
import omni
import carb
import asyncio # For robust waiting

LIDAR_USD_URL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Sensors/Ouster/OS1/OS1_REV6_128_10hz___512_resolution.usd"


def attach_lidar_to_drone(drone_prim_path: str, lidar_name: str, lidar_usd: str, lidar_offset: list[float], frame_id: str):
    """
    Attach a 360° LiDAR USD to a drone prim and copy its internal sensor prim
    so it appears locally (URDF export compatible).
    Also, connects the lidar as a fixed joint to the drone.

    Args:
        drone_prim_path (str): Path to the drone prim (e.g., "/World/Drone_01").
        lidar_name (str): Name of the lidar prim to create under the drone.
        lidar_usd (str): URL to the lidar USD file.
        lidar_offset (list[float]): Local [x, y, z] offset relative to drone.
        frame_id (str): Unique identifier for the sensor prim within the LiDAR.
    """

    stage = omni.usd.get_context().get_stage()
    lidar_prim_path = Sdf.Path(f"{drone_prim_path}/{lidar_name}")
    temp_prim_path = Sdf.Path(f"/_temp_lidar_source_{frame_id}")
    
    # --- Cleanup any existing temporary or target prims for a clean run ---
    if stage.GetPrimAtPath(temp_prim_path).IsValid():
        carb.log_info(f"Deleting existing temporary prim: {temp_prim_path}")
        omni.kit.commands.execute("DeletePrim", path=temp_prim_path)
        omni.kit.app.get_app().update() # Process delete
    
    if stage.GetPrimAtPath(lidar_prim_path).IsValid():
        carb.log_info(f"Deleting existing LiDAR instance at: {lidar_prim_path}")
        omni.kit.commands.execute("DeletePrim", path=lidar_prim_path)
        omni.kit.app.get_app().update() # Process delete


    # --- 1. Load the LiDAR USD temporarily to copy from ---
    carb.log_info(f"Defining temporary prim '{temp_prim_path}' and referencing '{lidar_usd}'")
    temp_prim = define_prim(str(temp_prim_path), "Xform")
    temp_prim.GetReferences().AddReference(lidar_usd)
    
    omni.usd.get_context().get_stage().Reload() 

    # Wait for the temporary prim and its child 'sensor' to be valid and resolved.
    sensor_temp_prim_path = Sdf.Path(f"{temp_prim_path}/sensor")
    carb.log_info(f"Waiting for temporary prim '{temp_prim_path}' and its 'sensor' child to load...")
    for _ in range(20): # Increased attempts for robustness
        omni.kit.app.get_app().update() # Process UI events
        if stage.GetPrimAtPath(temp_prim_path).IsValid() and stage.GetPrimAtPath(sensor_temp_prim_path).IsValid():
            carb.log_info(f"Temporary prim and sensor child loaded successfully.")
            break
        asyncio.sleep(0.01) # Short delay
    else:
        carb.log_error(f"Failed to load temporary LiDAR prim from '{lidar_usd}' and its 'sensor' child after multiple attempts. Cannot copy.")
        if stage.GetPrimAtPath(temp_prim_path).IsValid():
            omni.kit.commands.execute("DeletePrim", path=temp_prim_path)
        return None

    # --- 2. Copy the LiDAR prim under the drone ---
    carb.log_info(f"Copying prim from '{temp_prim_path}' to '{lidar_prim_path}'")
    omni.kit.commands.execute(
        "CopyPrim",
        path_from=temp_prim_path,
        path_to=lidar_prim_path, # Sdf.Path() is often implicit for this argument
        duplicate_layers=True,
    )

    # --- 3. Delete the temporary prim immediately after copy ---
    carb.log_info(f"Deleting temporary prim '{temp_prim_path}' after copy.")
    # omni.kit.commands.execute("DeletePrim", path=temp_prim_path)
    omni.usd.commands.DeletePrimsCommand([str(temp_prim_path)]).do()
    omni.kit.app.get_app().update() # Ensure deletion is processed

    # --- 4. Get the newly copied LiDAR prim and ensure it's valid ---
    # This block is now crucial to wait for the copied prim to appear
    carb.log_info(f"Waiting for copied LiDAR prim '{lidar_prim_path}' to become valid...")
    lidar_prim = None
    for _ in range(20):
        omni.kit.app.get_app().update()
        lidar_prim = stage.GetPrimAtPath(lidar_prim_path)
        if lidar_prim.IsValid():
            carb.log_info(f"Copied LiDAR prim '{lidar_prim_path}' is now valid.")
            break
        asyncio.sleep(0.01)
    else:
        carb.log_error(f"LiDAR prim '{lidar_prim_path}' is not valid after copying. Something went wrong.")
        return None
    
    # Ensure the LiDAR prim itself is a rigid body for the joint
    if not UsdPhysics.RigidBodyAPI(lidar_prim):
        carb.log_info(f"Applying RigidBodyAPI to LiDAR prim '{lidar_prim.GetPath()}'")
        UsdPhysics.RigidBodyAPI.Apply(lidar_prim)
    
    # Remove colliders if not needed for the LiDAR base transform itself
    physx_utils.removeCollider(lidar_prim)

    # --- 5. Apply local translation offset ---
    xform = UsdGeom.Xformable(lidar_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*lidar_offset))

    # --- 6. Rename the internal sensor prim ---
    sensor_prim_path_old = Sdf.Path(f"{lidar_prim_path}/sensor") # Original name
    sensor_prim_path_new = Sdf.Path(f"{lidar_prim_path}/{frame_id}")  # Desired new name

    # THIS IS THE CRITICAL WAIT FOR THE COPIED CHILDREN
    carb.log_info(f"Waiting for sensor prim '{sensor_prim_path_old}' under copied LiDAR to become valid/modifiable...")
    sensor_prim_old = None
    for _ in range(20):
        omni.kit.app.get_app().update()
        sensor_prim_old = stage.GetPrimAtPath(sensor_prim_path_old)
        if sensor_prim_old.IsValid():
            carb.log_info(f"Sensor prim '{sensor_prim_path_old}' is now valid.")
            break
        asyncio.sleep(0.01)
    else:
        carb.log_error(f"Sensor prim '{sensor_prim_path_old}' not valid/modifiable after copying LiDAR. Cannot rename.")
        return None # Indicate failure

    if sensor_prim_old.IsValid(): # Double check after loop
        carb.log_info(f"Renaming sensor prim from '{sensor_prim_path_old}' to '{sensor_prim_path_new}'")
        # omni.kit.commands.execute(
        #     "MovePrim",
        #     path_from=sensor_prim_old.GetPath(),
        #     path_to=sensor_prim_path_new,
        # )
        omni.kit.commands.execute(
            "CopyPrim",
            path_from=sensor_prim_old.GetPath(),
            path_to=sensor_prim_path_new, # Sdf.Path() is often implicit for this argument
            duplicate_layers=True,
        )
        omni.usd.commands.DeletePrimsCommand([str(sensor_prim_old.GetPath())]).do()
        omni.kit.app.get_app().update() # Ensure deletion is processed
    else:
        # This else branch should ideally not be hit if the loop above completed successfully
        carb.log_warning(f"Sensor prim '{sensor_prim_path_old}' not found after copying LiDAR and waiting. Unexpected state.")

    # --- 7. Create the Fixed Joint ---
    carb.log_info(f"Attempting to create Fixed Joint for LiDAR '{lidar_prim_path}'")
    
    drone_prim = stage.GetPrimAtPath(drone_prim_path) # Re-fetch drone_prim as it might have changed
    if not drone_prim.IsValid():
        carb.log_error(f"Drone prim '{drone_prim_path}' is not valid. Cannot create joint.")
        return None

    # Assuming 'body/body' is a valid rigid body child of the drone
    drone_body_prim_path = Sdf.Path(f"{drone_prim_path}/body/body")
    drone_body_prim = stage.GetPrimAtPath(drone_body_prim_path)

    if not drone_body_prim.IsValid():
        carb.log_error(f"Drone body prim '{drone_body_prim_path}' is not valid. Cannot create joint.")
        return None

    joint_prim_usd = physx_utils.createJoint(
        stage,
        joint_type="Fixed",
        from_prim=drone_body_prim,
        to_prim=lidar_prim,
    )

    if joint_prim_usd and joint_prim_usd.IsValid():
        carb.log_info(f"Successfully created Fixed Joint at '{joint_prim_usd.GetPath()}'")
    else:
        carb.log_error(f"Failed to create Fixed Joint for LiDAR. Joint prim returned as invalid.")
        return None
    # --- END JOINT CREATION ---

    carb.log_info(f"LiDAR '{lidar_name}' attached to '{drone_prim_path}' at offset {lidar_offset} with a fixed joint.")
    
    # Return the path to the copied/renamed local sensor prim for the rendering subgraph
    return str(sensor_prim_path_new)




def add_ouster_lidar_subgraph(parent_graph_handle: og._omni_graph_core.Graph,
                       drone_prim: str,
                       lidar_name: str = "OS1_REV6_128_10hz___512_resolution",
                       lidar_topic_name: str = "point_cloud",
                       lidar_usd: str = LIDAR_USD_URL,
                       lidar_offset: list = [0.0, 0.0, 0.025],
                       lidar_topic_namespace: str = "sensors/lidar",
                       lidar_frame_id: str = "lidar",
                       frame_height: int = 720,
                       frame_width: int = 1280,
                       robot_name: str = "robot_1",
                       domain_id: int = 1):
    
    controller = og.Controller()
    lidar_prim_path = attach_lidar_to_drone(drone_prim, lidar_name, lidar_usd, lidar_offset, lidar_frame_id)

    parent_graph_path = parent_graph_handle.get_path_to_graph()
    lidar_subgraph_name = f"{lidar_name}Graph"
    lidar_subgraph_path = f"{parent_graph_path}/{lidar_subgraph_name}/Subgraph"

    # Node names
    playback_tick = f"{lidar_name}PlaybackTick"
    ros2_context = f"{lidar_name}ROS2ContextNode"
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
                            # (ros2_context, "isaacsim.ros2.bridge.ROS2Context"),
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
                            (("inputs:value", frame_const), lidar_name),
                            (("inputs:value", ns_const), lidar_topic_namespace),
                            (("inputs:cameraPrim", create_render), lidar_prim_path),
                            (("inputs:height", create_render), frame_height),
                            (("inputs:width", create_render), frame_width),
                            # (("inputs:domain_id", ros2_context), domain_id),
                            (("inputs:topicName", rtx_helper), f"{lidar_topic_name}"),
                            (("inputs:type", rtx_helper), f"point_cloud"),
                            (("inputs:nodeNamespace", rtx_helper), f"{robot_name}/{lidar_topic_namespace}"),
                        ],
                        og.Controller.Keys.CONNECT: [
                            # Simulation frame tick
                            (f"{playback_tick}.outputs:tick", f"{run_one_sim_frame}.inputs:execIn"),
                            (f"{run_one_sim_frame}.outputs:step", f"{create_render}.inputs:execIn"),

                            (f"{create_render}.outputs:execOut", f"{rtx_helper}.inputs:execIn"),
                            
                            # Render product to RTX Helper
                            (f"{create_render}.outputs:renderProductPath", f"{rtx_helper}.inputs:renderProductPath"),
                            # (f"{ros2_context}.outputs:context", f"{rtx_helper}.inputs:context"),
                            (f"{frame_const}.inputs:value", f"{rtx_helper}.inputs:frameId"),
                            (f"{ns_const}.inputs:value", f"{rtx_helper}.inputs:nodeNamespace"),

                        ]
                    },
                )
            ]
        },
    )

    print(f"LiDAR subgraph '{lidar_subgraph_name}' added under '{parent_graph_path}'.")


# Example usage
# add_lidar_subgraph(my_graph_handle, "/World/Drone_01")

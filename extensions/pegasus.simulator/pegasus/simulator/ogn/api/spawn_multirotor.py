import omni.graph.core as og
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from omni.usd import get_stage_next_free_path
from pegasus.simulator.ogn.api.shared_node_names import *
from pxr import UsdGeom, Gf

def spawn_px4_multirotor_node(
    pegasus_node_name: str = "PX4Multirotor",
    graph_name: str = "PX4MultirotorGraph",
    drone_prim: str = "/World/Quadrotor",
    usd_file: str = "/root/Documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd",
    robot_name: str = "robot_1",
    vehicle_id: int = 0,
    domain_id: int = 1,
    init_pos: list = [0.0, 0.0, 0.07],
    init_orient: list = [0.0, 0.0, 0.0, 1.0],
    connection_type: str = "tcpin",
    connection_ip: str = "localhost",
    connection_baseport: int = 4560,
    px4_autolaunch: bool = True,
    px4_dir: str = "/root/PX4-Autopilot",
    px4_vehicle_model: str = "gazebo-classic_iris",
    enable_lockstep: bool = True,
    num_rotors: int = 4,
    update_rate: float = 250.0,
    input_offset_0: float = 0.0,
    input_offset_1: float = 0.0,
    input_offset_2: float = 0.0,
    input_offset_3: float = 0.0,
    input_scaling_0: float = 1000.0,
    input_scaling_1: float = 1000.0,
    input_scaling_2: float = 1000.0,
    input_scaling_3: float = 1000.0,
    zero_position_armed_0: float = 100.0,
    zero_position_armed_1: float = 100.0,
    zero_position_armed_2: float = 100.0,
    zero_position_armed_3: float = 100.0,
    robot_name_var_name: str = "robot_name_var",
    domain_id_var_name: str = "domain_id_var",
    vehicle_id_var_name: str = "vehicle_id_var",
    sensors_topic_var_name: str = "sensors_topic_var",
    sensors_topic_namespace: str = "sensors",
):
    """
    Spawn a PX4 multirotor node in an on-demand graph and place its USD prim
    at the specified initial position and orientation.
    """
    # Unpack initial position and orientation
    init_pos_x, init_pos_y, init_pos_z = init_pos
    init_orient_x, init_orient_y, init_orient_z, init_orient_w = init_orient

    # Get a free path for the drone prim
    drone_prim = get_stage_next_free_path(PegasusInterface().world.stage, drone_prim, False)

    # Spawn prim if missing
    prim = get_prim_at_path(drone_prim)
    if not prim.IsValid():
        prim = define_prim(drone_prim, "Xform")
        prim.GetReferences().AddReference(usd_file)
        # Apply initial transform to Xform prim
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        translate_op = xform.AddTranslateOp()
        orient_op = xform.AddOrientOp()
        translate_op.Set(Gf.Vec3d(init_pos_x, init_pos_y, init_pos_z))
        orient_op.Set(Gf.Quatf(init_orient_w, init_orient_x, init_orient_y, init_orient_z))
        xform.SetXformOpOrder([translate_op, orient_op])
        print(f"Prim '{drone_prim}' created at initial position {init_pos} and orientation {init_orient}.")
    else:
        print(f"Prim at {drone_prim} already exists, reusing.")

    graph_path = f"{drone_prim}/{graph_name}"
    px4_node_path = f"{graph_path}/{pegasus_node_name}"

    playbackTick_node_name = f"{robot_name}_PlaybackTick"
    physicsStep_node_name = f"{robot_name}_PhysicsStep"
    ros2Context_node_name = f"{robot_name}_ROS2Context"
    ros2PublishClock_node_name = f"{robot_name}_ROS2PublishClock"
    isaacReadSimTime_node_name = f"{robot_name}_IsaacReadSimTime"
    getPrimPath_node_name = f"{robot_name}_GetPrimPath"
    robotNameReader_node_name = f"{robot_name}_RobotName"
    domainIDReader_node_name = f"{robot_name}_DomainIDReader"
    vehicleIDReader_node_name = f"{robot_name}_VehicleIDReader"

    # Create on-demand graph with nodes using Controller.edit
    graph_handle, _, _, _ = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND
        },
        {
            og.Controller.Keys.CREATE_VARIABLES: [
                (robot_name_var_name, "string", robot_name),
                (domain_id_var_name, og.Type(og.BaseDataType.UCHAR), domain_id),
                (vehicle_id_var_name, og.Type(og.BaseDataType.INT), vehicle_id),
                (sensors_topic_var_name, "string", sensors_topic_namespace),
            ],
            og.Controller.Keys.CREATE_NODES: [
                # Variables
                (domainIDReader_node_name, "omni.graph.core.ReadVariable"),
                (vehicleIDReader_node_name, "omni.graph.core.ReadVariable"),
                # Pegasus PX4 Backend Node
                (pegasus_node_name, "pegasus.simulator.PegasusMultirotorPX4Node"),
                (getPrimPath_node_name, "omni.graph.nodes.GetPrimPath"),
                # Synchronization and ROS2 Clock Publisher
                (playbackTick_node_name, "omni.graph.action.OnPlaybackTick"),
                (physicsStep_node_name, "isaacsim.core.nodes.OnPhysicsStep"),
                (isaacReadSimTime_node_name, "isaacsim.core.nodes.IsaacReadSimulationTime"),
                (ros2Context_node_name, "isaacsim.ros2.bridge.ROS2Context"),
                (ros2PublishClock_node_name, "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # PX4 inputs
                (f"{pegasus_node_name}.inputs:dronePrim", drone_prim),
                (f"{pegasus_node_name}.inputs:usdFile", usd_file),
                (f"{pegasus_node_name}.inputs:connectionType", connection_type),
                (f"{pegasus_node_name}.inputs:connectionIP", connection_ip),
                (f"{pegasus_node_name}.inputs:connectionBaseport", connection_baseport),
                (f"{pegasus_node_name}.inputs:px4Autolaunch", px4_autolaunch),
                (f"{pegasus_node_name}.inputs:px4Dir", px4_dir),
                (f"{pegasus_node_name}.inputs:px4VehicleModel", px4_vehicle_model),
                (f"{pegasus_node_name}.inputs:enableLockstep", enable_lockstep),
                (f"{pegasus_node_name}.inputs:numRotors", num_rotors),
                (f"{pegasus_node_name}.inputs:updateRate", update_rate),
                (f"{pegasus_node_name}.inputs:inputOffset0", input_offset_0),
                (f"{pegasus_node_name}.inputs:inputOffset1", input_offset_1),
                (f"{pegasus_node_name}.inputs:inputOffset2", input_offset_2),
                (f"{pegasus_node_name}.inputs:inputOffset3", input_offset_3),
                (f"{pegasus_node_name}.inputs:inputScaling0", input_scaling_0),
                (f"{pegasus_node_name}.inputs:inputScaling1", input_scaling_1),
                (f"{pegasus_node_name}.inputs:inputScaling2", input_scaling_2),
                (f"{pegasus_node_name}.inputs:inputScaling3", input_scaling_3),
                (f"{pegasus_node_name}.inputs:zeroPositionArmed0", zero_position_armed_0),
                (f"{pegasus_node_name}.inputs:zeroPositionArmed1", zero_position_armed_1),
                (f"{pegasus_node_name}.inputs:zeroPositionArmed2", zero_position_armed_2),
                (f"{pegasus_node_name}.inputs:zeroPositionArmed3", zero_position_armed_3),
                # Prim path
                (f"{getPrimPath_node_name}.inputs:prim", "../.."),
                # Variable reader setup
                (f"{domainIDReader_node_name}.inputs:variableName", domain_id_var_name),
                (f"{vehicleIDReader_node_name}.inputs:variableName", vehicle_id_var_name),
            ],
            og.Controller.Keys.CONNECT: [
                (f"{physicsStep_node_name}.outputs:step", f"{pegasus_node_name}.inputs:execIn"),
                (f"{physicsStep_node_name}.outputs:step", f"{ros2PublishClock_node_name}.inputs:execIn"),
                (f"{isaacReadSimTime_node_name}.outputs:simulationTime", f"{ros2PublishClock_node_name}.inputs:timeStamp"),
                (f"{ros2Context_node_name}.outputs:context", f"{ros2PublishClock_node_name}.inputs:context"),
                (f"{getPrimPath_node_name}.outputs:path", f"{pegasus_node_name}.inputs:dronePrim"),
                # Variable connections
                (f"{domainIDReader_node_name}.outputs:value", f"{ros2Context_node_name}.inputs:domain_id"),
                (f"{vehicleIDReader_node_name}.outputs:value", f"{pegasus_node_name}.inputs:vehicleID"),
            ],
        }
    )


    print(f"PX4 node at {px4_node_path} created in on-demand graph.")
    return graph_handle
import omni.graph.core as og
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from omni.usd import get_stage_next_free_path
from pegasus.simulator.ogn.api.shared_node_names import *

def spawn_px4_multirotor_node(
    pegasus_node_name: str = "PX4Multirotor",
    graph_name: str = "PX4MultirotorGraph",
    drone_prim: str = "/World/Quadrotor",
    usd_file: str = "/root/Documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd",
    robot_name: str = "robot_1",

    # Vehicle config
    vehicle_id: int = 0,
    domain_id: int = 1,

    # Initial pose
    init_pos: list = [0.0, 0.0, 0.07],
    init_orient: list = [0.0, 0.0, 0.0, 1.0],

    # Connection config
    connection_type: str = "tcpin",
    connection_ip: str = "localhost",
    connection_baseport: int = 4560,

    # PX4 config
    px4_autolaunch: bool = True,
    px4_dir: str = "/root/PX4-Autopilot",
    px4_vehicle_model: str = "gazebo-classic_iris",

    # Simulation config
    enable_lockstep: bool = True,
    num_rotors: int = 4,
    update_rate: float = 250.0,

    # Rotor inputs
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
):
    
    # Unpack initial position and orientation
    init_pos_x: float = init_pos[0]
    init_pos_y: float = init_pos[1]
    init_pos_z: float = init_pos[2]
    init_orient_x: float = init_orient[0]
    init_orient_y: float = init_orient[1]
    init_orient_z: float = init_orient[2]
    init_orient_w: float = init_orient[3]

    # Get a free path for the drone prim
    drone_prim = get_stage_next_free_path(PegasusInterface().world.stage, drone_prim, False)

    # --- Spawn prim if missing ---
    prim = get_prim_at_path(drone_prim)
    if not prim.IsValid():
        prim = define_prim(drone_prim, "Xform")
        prim.GetReferences().AddReference(usd_file)
    else:
        print(f"Prim at {drone_prim} already exists, reusing.")

    graph_path = f"{drone_prim}/{graph_name}"
    px4_node_path = f"{graph_path}/{pegasus_node_name}"


    # --- Create on-demand graph with nodes using Controller.edit ---
    graph_handle, _, _, _ = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                # Pegasus PX4 Backend Node
                (pegasus_node_name, "pegasus.simulator.PegasusMultirotorPX4Node"),
                (GET_PRIM_PATH_NODE, "omni.graph.nodes.GetPrimPath"),
                # Robot name constant
                (ROBOT_NAME_NODE, "omni.graph.nodes.ConstantString"),
                # Synchronization and ROS2 Clock Publisher
                (PLAYBACK_TICK_NODE, "omni.graph.action.OnPlaybackTick"),
                (ISAAC_READ_SIM_TIME_NODE, "isaacsim.core.nodes.IsaacReadSimulationTime"),
                (ROS2_CONTEXT_NODE, "isaacsim.ros2.bridge.ROS2Context"),
                (ROS2_PUBLISH_CLOCK_NODE, "isaacsim.ros2.bridge.ROS2PublishClock"),
                # Constants for initial pose
                (INIT_POSITION_NODE, "omni.graph.nodes.ConstantFloat3"),
                (INIT_ORIENTATION_NODE, "omni.graph.nodes.ConstantFloat4"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # --- PX4 inputs that are constants ---
                ("InitPosition.inputs:value", (init_pos_x, init_pos_y, init_pos_z)),
                ("InitOrientation.inputs:value", (init_orient_x, init_orient_y, init_orient_z, init_orient_w)),
                # --- PX4 inputs ---
                (f"{pegasus_node_name}.inputs:dronePrim", drone_prim),
                (f"{pegasus_node_name}.inputs:vehicleID", vehicle_id),
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
                # ROS2 Context
                (f"{ROS2_CONTEXT_NODE}.inputs:domain_id", domain_id),
                # Prim path
                (f"{GET_PRIM_PATH_NODE}.inputs:prim", "../.."),
                # Robot name
                (f"{ROBOT_NAME_NODE}.inputs:value", robot_name),
            ],
            og.Controller.Keys.CONNECT: [
                (f"{PLAYBACK_TICK_NODE}.outputs:tick", f"{pegasus_node_name}.inputs:execIn"),
                (f"{PLAYBACK_TICK_NODE}.outputs:tick", f"{ROS2_PUBLISH_CLOCK_NODE}.inputs:execIn"),
                (f"{ISAAC_READ_SIM_TIME_NODE}.outputs:simulationTime", f"{ROS2_PUBLISH_CLOCK_NODE}.inputs:timeStamp"),
                (f"{ROS2_CONTEXT_NODE}.outputs:context", f"{ROS2_PUBLISH_CLOCK_NODE}.inputs:context"),
                (f"{GET_PRIM_PATH_NODE}.outputs:path", f"{pegasus_node_name}.inputs:dronePrim"),
                # --- Initial Position ---
                (f"{INIT_POSITION_NODE}.inputs:value", f"{pegasus_node_name}.inputs:initPos"),
                (f"{INIT_ORIENTATION_NODE}.inputs:value", f"{pegasus_node_name}.inputs:initOrient"),
            ],
        }
    )


    print(f"PX4 node at {px4_node_path} created in on-demand graph.")
    return graph_handle
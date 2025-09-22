import omni.graph.core as og
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from omni.usd import get_stage_next_free_path


def spawn_px4_multirotor_node(
    node_name: str = "PX4Multirotor",
    graph_name: str = "PX4MultirotorGraph",
    drone_prim: str = "/World/Quadrotor",
    usd_file: str = "/root/Documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd",

    # Vehicle config
    vehicle_id: int = 0,
    domain_id: int = 1,

    # Initial pose
    init_pos_x: float = 0.0,
    init_pos_y: float = 0.0,
    init_pos_z: float = 0.07,
    init_orient_x: float = 0.0,
    init_orient_y: float = 0.0,
    init_orient_z: float = 0.0,
    init_orient_w: float = 1.0,

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
    px4_node_path = f"{graph_path}/{node_name}"
    playback_tick_node_path = f"{graph_path}/OnPlaybackTick"

    # --- Create on-demand graph with nodes using Controller.edit ---
    demand_graph_handle, _, _, _ = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                (node_name, "pegasus.simulator.PegasusMultirotorPX4Node"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("get_prim_path", "omni.graph.nodes.GetPrimPath"),
                ("IsaacReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ROS2PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # PX4 inputs
                (f"{node_name}.inputs:dronePrim", drone_prim),
                (f"{node_name}.inputs:vehicleID", vehicle_id),
                (f"{node_name}.inputs:usdFile", usd_file),
                (f"{node_name}.inputs:initPosX", init_pos_x),
                (f"{node_name}.inputs:initPosY", init_pos_y),
                (f"{node_name}.inputs:initPosZ", init_pos_z),
                (f"{node_name}.inputs:initOrientX", init_orient_x),
                (f"{node_name}.inputs:initOrientY", init_orient_y),
                (f"{node_name}.inputs:initOrientZ", init_orient_z),
                (f"{node_name}.inputs:initOrientW", init_orient_w),
                (f"{node_name}.inputs:connectionType", connection_type),
                (f"{node_name}.inputs:connectionIP", connection_ip),
                (f"{node_name}.inputs:connectionBaseport", connection_baseport),
                (f"{node_name}.inputs:px4Autolaunch", px4_autolaunch),
                (f"{node_name}.inputs:px4Dir", px4_dir),
                (f"{node_name}.inputs:px4VehicleModel", px4_vehicle_model),
                (f"{node_name}.inputs:enableLockstep", enable_lockstep),
                (f"{node_name}.inputs:numRotors", num_rotors),
                (f"{node_name}.inputs:updateRate", update_rate),
                (f"{node_name}.inputs:inputOffset0", input_offset_0),
                (f"{node_name}.inputs:inputOffset1", input_offset_1),
                (f"{node_name}.inputs:inputOffset2", input_offset_2),
                (f"{node_name}.inputs:inputOffset3", input_offset_3),
                (f"{node_name}.inputs:inputScaling0", input_scaling_0),
                (f"{node_name}.inputs:inputScaling1", input_scaling_1),
                (f"{node_name}.inputs:inputScaling2", input_scaling_2),
                (f"{node_name}.inputs:inputScaling3", input_scaling_3),
                (f"{node_name}.inputs:zeroPositionArmed0", zero_position_armed_0),
                (f"{node_name}.inputs:zeroPositionArmed1", zero_position_armed_1),
                (f"{node_name}.inputs:zeroPositionArmed2", zero_position_armed_2),
                (f"{node_name}.inputs:zeroPositionArmed3", zero_position_armed_3),
                # ROS2 Context
                ("ROS2Context.inputs:domain_id", domain_id),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", f"{node_name}.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", f"ROS2PublishClock.inputs:execIn"),
                ("IsaacReadSimTime.outputs:simulationTime", "ROS2PublishClock.inputs:timeStamp"),
                ("ROS2Context.outputs:context", "ROS2PublishClock.inputs:context"),
            ],
        }
    )

    # demand_graph_handle.evaluate() # Trigger graph evaluation to apply changes

    print(f"PX4 node at {px4_node_path} created in on-demand graph.")
    return demand_graph_handle
import omni.graph.core as og
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from omni.usd import get_stage_next_free_path

def omnigraph_node_exists(node_path: str) -> bool:
    try:
        og.Controller.node(node_path)
        return True
    except og.OmniGraphValueError:
        return False


def spawn_px4_multirotor_node(
    node_name: str = "PX4Multirotor",
    drone_prim: str = "/World/Quadrotor",
    usd_file: str = "/root/Documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd",

    # Vehicle config
    vehicle_id: int = 0,

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
    """
    Spawn a PegasusPX4MultirotorNode in OmniGraph with OnTick,
    setting all inputs with default values. Safe to call multiple times.
    """
    drone_prim = get_stage_next_free_path(PegasusInterface().world.stage, drone_prim, False)

    # --- Spawn the prim if missing ---
    prim = get_prim_at_path(drone_prim)
    if not prim.IsValid():
        prim = define_prim(drone_prim, "Xform")
        prim.GetReferences().AddReference(usd_file)
    else:
        print(f"Prim at {drone_prim} already exists, reusing.")
    # Absolute graph path inside the prim
    graph_path = f"{drone_prim}/{node_name}"
    px4_node_path = f"{graph_path}/{node_name}"
    on_tick_node_path = f"{graph_path}/OnTick"
    # absolute_graph_path = graph_path

    og.Controller.edit(
        # {"graph_path": px4_node_path.rsplit("/", 1)[0]},
        {"graph_path": graph_path},
        {og.Controller.Keys.CREATE_NODES: [
            (node_name, "pegasus.simulator.PegasusPX4MultirotorNode"), 
            # ("OnTick", "omni.graph.action.OnTick"),
        ]}
    )

    # --- Set all input attributes ---
    defaults = {
        # Vehicle config
        "inputs:dronePrim": drone_prim,
        "inputs:vehicleID": vehicle_id,
        "inputs:usdFile": usd_file,

        # Initial pose
        "inputs:initPosX": init_pos_x,
        "inputs:initPosY": init_pos_y,
        "inputs:initPosZ": init_pos_z,
        "inputs:initOrientX": init_orient_x,
        "inputs:initOrientY": init_orient_y,
        "inputs:initOrientZ": init_orient_z,
        "inputs:initOrientW": init_orient_w,

        # Connection config
        "inputs:connectionType": connection_type,
        "inputs:connectionIP": connection_ip,
        "inputs:connectionBaseport": connection_baseport,

        # PX4 config
        "inputs:px4Autolaunch": px4_autolaunch,
        "inputs:px4Dir": px4_dir,
        "inputs:px4VehicleModel": px4_vehicle_model,

        # Simulation config
        "inputs:enableLockstep": enable_lockstep,
        "inputs:numRotors": num_rotors,
        "inputs:updateRate": update_rate,

        # Rotor inputs
        "inputs:inputOffset0": input_offset_0,
        "inputs:inputOffset1": input_offset_1,
        "inputs:inputOffset2": input_offset_2,
        "inputs:inputOffset3": input_offset_3,
        "inputs:inputScaling0": input_scaling_0,
        "inputs:inputScaling1": input_scaling_1,
        "inputs:inputScaling2": input_scaling_2,
        "inputs:inputScaling3": input_scaling_3,
        "inputs:zeroPositionArmed0": zero_position_armed_0,
        "inputs:zeroPositionArmed1": zero_position_armed_1,
        "inputs:zeroPositionArmed2": zero_position_armed_2,
        "inputs:zeroPositionArmed3": zero_position_armed_3,
    }

    for attr, val in defaults.items():
        og.Controller.set(f"{px4_node_path}.{attr}", val)

    # # --- Connect OnTick to PX4 execIn ---
    # og.Controller.connect(f"{on_tick_node_path}.outputs:tick", f"{px4_node_path}.inputs:execIn")

    print(f"PX4 node at {px4_node_path} ready with OnTick driver.")
    return px4_node_path

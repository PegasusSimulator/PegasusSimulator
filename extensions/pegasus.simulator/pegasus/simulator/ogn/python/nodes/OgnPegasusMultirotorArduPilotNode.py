"""
ArduPilot-specific Pegasus Multirotor OmniGraph node.
Extends the base multirotor node with ArduPilot-specific backend configuration.
"""

import sys
import traceback
import omni.graph.core as og
import omni.graph.tools.ogn as ogn

from pegasus.simulator.logic.backends import ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig

from pegasus.simulator.ogn.OgnPegasusMultirotorArduPilotNodeDatabase import OgnPegasusMultirotorArduPilotNodeDatabase
from pegasus.simulator.ogn.python.nodes.OgnPegasusMultirotorNodeBase import OgnPegasusMultirotorNodeBase


class OgnPegasusMultirotorArduPilotNodeState:
    def __init__(self):
        self.node_initialized: bool = False


class OgnPegasusMultirotorArduPilotNode:
    """ArduPilot-specific Pegasus Multirotor OmniGraph node"""
    
    @staticmethod
    def internal_state():
        return OgnPegasusMultirotorArduPilotNodeState()

    @staticmethod
    def get_node_type():
        return "pegasus.simulator.PegasusMultirotorArduPilotNode"

    @staticmethod
    def initialize(context, node: og.Node):
        """Initialize the ArduPilot node"""
        return OgnPegasusMultirotorNodeBase.initialize(context, node, OgnPegasusMultirotorArduPilotNodeDatabase)

    @staticmethod
    def release(node: og.Node):
        """Release node resources"""
        return OgnPegasusMultirotorNodeBase.release(node)

    @staticmethod
    def release_instance(node, graph_instance_id):
        """Release instance resources"""
        return OgnPegasusMultirotorNodeBase.release_instance(node, graph_instance_id)

    @staticmethod
    def create_ardupilot_backend_config(db):
        """Create ArduPilot-specific backend configuration with explicit parameter mapping"""
        # Explicitly map each OmniGraph input to backend configuration parameter
        config_dict = {
            # Vehicle identification
            "vehicle_id": db.inputs.vehicleID,
            
            # MAVLink connection configuration
            "connection_type": db.inputs.connectionType,        # e.g., "tcpin", "udp"
            "connection_ip": db.inputs.connectionIP,            # e.g., "localhost"
            "connection_baseport": db.inputs.connectionBaseport, # e.g., 4560
            
            # ArduPilot-specific autolaunch settings
            "ardupilot_autolaunch": db.inputs.ardupilotAutolaunch,      # True/False - launch ArduPilot automatically
            "ardupilot_dir": db.inputs.ardupilotDir,                    # Path to ArduPilot installation
            "ardupilot_vehicle_model": db.inputs.ardupilotVehicleModel, # ArduPilot vehicle model name
            
            # Simulation configuration
            "enable_lockstep": db.inputs.enableLockstep,        # True/False - synchronize simulation
            "num_rotors": db.inputs.numRotors,                  # Number of rotors (typically 4)
            "update_rate": db.inputs.updateRate,                # Backend update frequency in Hz
            
            # Per-rotor input configuration arrays
            "input_offset": [
                db.inputs.inputOffset0,                         # Rotor 0 input offset
                db.inputs.inputOffset1,                         # Rotor 1 input offset
                db.inputs.inputOffset2,                         # Rotor 2 input offset
                db.inputs.inputOffset3                          # Rotor 3 input offset
            ],
            "input_scaling": [
                db.inputs.inputScaling0,                        # Rotor 0 input scaling factor
                db.inputs.inputScaling1,                        # Rotor 1 input scaling factor
                db.inputs.inputScaling2,                        # Rotor 2 input scaling factor
                db.inputs.inputScaling3                         # Rotor 3 input scaling factor
            ],
            # ArduPilot-specific: Input min/max constraints
            "input_min": [
                db.inputs.inputMin0,                            # Rotor 0 minimum input value
                db.inputs.inputMin1,                            # Rotor 1 minimum input value
                db.inputs.inputMin2,                            # Rotor 2 minimum input value
                db.inputs.inputMin3                             # Rotor 3 minimum input value
            ],
            "input_max": [
                db.inputs.inputMax0,                            # Rotor 0 maximum input value
                db.inputs.inputMax1,                            # Rotor 1 maximum input value
                db.inputs.inputMax2,                            # Rotor 2 maximum input value
                db.inputs.inputMax3                             # Rotor 3 maximum input value
            ],
            "zero_position_armed": [
                db.inputs.zeroPositionArmed0,                   # Rotor 0 zero position when armed
                db.inputs.zeroPositionArmed1,                   # Rotor 1 zero position when armed
                db.inputs.zeroPositionArmed2,                   # Rotor 2 zero position when armed
                db.inputs.zeroPositionArmed3                    # Rotor 3 zero position when armed
            ]
        }
        
        print(f"ArduPilot Backend Configuration:")
        print(f"  Vehicle ID: {config_dict['vehicle_id']}")
        print(f"  Connection: {config_dict['connection_type']}://{config_dict['connection_ip']}:{config_dict['connection_baseport']}")
        print(f"  ArduPilot Autolaunch: {config_dict['ardupilot_autolaunch']}")
        print(f"  ArduPilot Directory: {config_dict['ardupilot_dir']}")
        print(f"  ArduPilot Vehicle Model: {config_dict['ardupilot_vehicle_model']}")
        print(f"  Lockstep Enabled: {config_dict['enable_lockstep']}")
        print(f"  Number of Rotors: {config_dict['num_rotors']}")
        print(f"  Update Rate: {config_dict['update_rate']} Hz")
        
        return ArduPilotMavlinkBackendConfig(config_dict)

    @staticmethod
    def compute(db) -> bool:
        """Main compute method that handles ArduPilot drone spawning and execution"""
        try:
            # Use base compute functionality with ArduPilot-specific backend
            return OgnPegasusMultirotorNodeBase.compute_base(
                db, 
                ArduPilotMavlinkBackend, 
                ArduPilotMavlinkBackendConfig,
                OgnPegasusMultirotorArduPilotNode.create_ardupilot_backend_config
            )
                    
        except Exception as e:
            print('Error in ArduPilot compute method:', e)
            traceback.print_exc()
            OgnPegasusMultirotorNodeBase._print_stacktrace(db)
            return False

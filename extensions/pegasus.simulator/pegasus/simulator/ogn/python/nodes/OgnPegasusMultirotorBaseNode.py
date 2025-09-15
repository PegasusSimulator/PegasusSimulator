"""
Base class for Pegasus Multirotor OmniGraph nodes.
Provides common functionality for drone spawning and management.
"""

import inspect
import os
import traceback
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import carb
import omni
import omni.client
import omni.graph.core as og
import omni.graph.tools.ogn as ogn
import omni.usd
import omni.replicator.core as rep
import omni.timeline
import usdrt.Sdf
from omni.isaac.core.prims import GeometryPrim, RigidPrim
from omni.isaac.core.utils import extensions, stage
from omni.isaac.core.world import World
from pxr import Gf, Usd, UsdGeom
import time
import threading
import struct
import socket
import asyncio

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, BACKENDS, WORLD_SETTINGS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends import Backend, BackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.vehicle_manager import VehicleManager
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

# Global variables for drone simulation tracking
drone_sim_dict = {}
initialized_timeline_callback = False


def timeline_callback(event):
    """Timeline callback to handle simulation events"""
    global drone_sim_dict
    print("pegasus multirotor base node timeline callback", event, event.type, dir(event))

    if event.type == int(omni.timeline.TimelineEventType.PLAY):
        pass
    elif event.type == int(omni.timeline.TimelineEventType.PAUSE):
        pass
    elif event.type == int(omni.timeline.TimelineEventType.STOP):
        # Reset orientation and position of the drone
        for k, v in drone_sim_dict.items():
            if 'multirotor' in v and hasattr(v['multirotor'], '_prim') and v['multirotor']._prim:
                prim = v['multirotor']._prim
                if prim.GetAttribute("xformOp:orient").Get() is not None:
                    prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(1, 0, 0, 0))
                prim.GetAttribute("xformOp:translate").Set((0, 0, 0))

        drone_sim_dict = {}


class OgnPegasusMultirotorBaseNodeState:
    def __init__(self):
        self.node_initialized: bool = False  # Flag used to check if the per-instance node state is initialized.


class OgnPegasusMultirotorBaseNode:
    """Base class for Pegasus Multirotor OmniGraph nodes"""
    
    @staticmethod
    def internal_state():
        return OgnPegasusMultirotorBaseNodeState()

    @staticmethod
    def _is_initialized(node: og.Node) -> bool:
        return og.Controller.get(node.get_attribute("state:omni_initialized"))

    @staticmethod
    def _set_initialized(node: og.Node, init: bool):
        return og.Controller.set(node.get_attribute("state:omni_initialized"), init)

    @staticmethod
    def initialize(context, node: og.Node, db_class):
        """Initialize the node"""
        # Initialize state if shared_internal_state exists
        try:
            state = db_class.shared_internal_state(node)
            state.node_initialized = True
        except AttributeError:
            # If shared_internal_state doesn't exist, create a simple state
            pass

        OgnPegasusMultirotorBaseNode._set_initialized(node, False)

    @staticmethod
    def release(node: og.Node):
        """Release node resources"""
        # Clean up drone simulation
        OgnPegasusMultirotorBaseNode.try_cleanup(node)

    @staticmethod
    def release_instance(node, graph_instance_id):
        """Overrides the release_instance method so that any per-script cleanup can happen before the per-node data
        is deleted.
        """
        # Same logic as when the reset button is pressed
        OgnPegasusMultirotorBaseNode.try_cleanup(node)

    @staticmethod
    def try_cleanup(node: og.Node):
        """Clean up drone simulation resources"""
        global drone_sim_dict
        
        # Skip if not setup in the first place or already cleaned up
        if not OgnPegasusMultirotorBaseNode._is_initialized(node):
            return

        node_id = node.node_id()
        
        # Clean up drone simulation if it exists
        if node_id in drone_sim_dict:
            try:
                drone_data = drone_sim_dict[node_id]
                if 'backend' in drone_data and drone_data['backend']:
                    # Clean up backend if it has cleanup methods
                    if hasattr(drone_data['backend'], 'cleanup'):
                        drone_data['backend'].cleanup()
                    
                if 'multirotor' in drone_data and drone_data['multirotor']:
                    # Clean up multirotor if it has cleanup methods
                    if hasattr(drone_data['multirotor'], 'cleanup'):
                        drone_data['multirotor'].cleanup()
                        
                del drone_sim_dict[node_id]
                print(f"Cleaned up drone simulation for node {node_id}")
                
            except Exception as e:
                print(f"Error during cleanup for node {node_id}: {e}")
                traceback.print_exc()
        
        OgnPegasusMultirotorBaseNode._set_initialized(node, False)

    @staticmethod
    def _print_stacktrace(db):
        """Print stacktrace for debugging"""
        stacktrace = traceback.format_exc().splitlines(keepends=True)
        stacktrace_iter = iter(stacktrace)
        stacktrace_output = ""

        for stacktrace_line in stacktrace_iter:
            if "OgnPegasusMultirotor" in stacktrace_line:
                # The stack trace shows that the exception originates from this file
                # Removing this useless information from the stack trace
                next(stacktrace_iter, None)
            else:
                stacktrace_output += stacktrace_line

        if hasattr(db, 'log_error'):
            db.log_error(stacktrace_output)
        else:
            print("Error:", stacktrace_output)

    @staticmethod
    def create_base_backend_config(db, backend_config_class):
        """Create base backend configuration from common inputs"""
        # This method will be overridden by specific backend implementations
        # since each backend has different configuration parameters
        raise NotImplementedError("This method should be overridden by specific backend nodes")

    @staticmethod
    def create_multirotor(db, backend):
        """Create multirotor vehicle with the given backend without validation for scalability"""
        # Create multirotor configuration
        multirotor_config = MultirotorConfig()
        multirotor_config.backends = [backend]

        # Get USD file from input parameter
        selected_usd_file = db.inputs.usdFile

        # Create multirotor vehicle directly without validation
        multirotor = Multirotor(
            stage_prefix=db.inputs.dronePrim,
            usd_file=selected_usd_file,  # Use USD file directly
            vehicle_id=0,  # This is used internally by multirotor, separate from backend vehicle_id
            init_pos=[db.inputs.initPosX, db.inputs.initPosY, db.inputs.initPosZ],
            init_orientation=[db.inputs.initOrientX, db.inputs.initOrientY, db.inputs.initOrientZ, db.inputs.initOrientW],
            config=multirotor_config,
            spawn_prim=False
        )
        
        return multirotor, multirotor_config

    @staticmethod
    def compute_base(db, backend_class, backend_config_class, create_config_func) -> bool:
        """Base compute method that handles common drone spawning and execution"""
        global drone_sim_dict, initialized_timeline_callback
        
        # Initialize timeline callback if not already done
        if not initialized_timeline_callback:
            initialized_timeline_callback = True
            timeline = omni.timeline.get_timeline_interface()
            timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.PLAY), timeline_callback
            )
            timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.PAUSE), timeline_callback
            )
            timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.STOP), timeline_callback
            )

        try:
            # Check if we have drone prim input
            if db.inputs.dronePrim:
                node_id = db.node.node_id()
                
                # Initialize drone if not already done
                if node_id not in drone_sim_dict.keys():
                    print(f'Creating new drone with VEHICLE ID: {db.inputs.vehicleID}')
                    
                    # Create backend configuration using the provided function
                    backend_config = create_config_func(db)
                    backend = backend_class(config=backend_config)

                    # Create multirotor vehicle
                    multirotor, multirotor_config = OgnPegasusMultirotorBaseNode.create_multirotor(db, backend)

                    # Store drone simulation data
                    drone_sim_dict[node_id] = {
                        'backend_config': backend_config,
                        'backend': backend,
                        'multirotor_config': multirotor_config,
                        'multirotor': multirotor,
                    }
                    
                    print(f"Successfully created drone simulation for node {node_id}")
                    
        except Exception as e:
            print('Error in compute method:', e)
            traceback.print_exc()
            return False

        # Set outputs:execOut if not hidden
        if db.node.get_attribute("outputs:execOut").get_metadata(ogn.MetadataKeys.HIDDEN) != "1":
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True

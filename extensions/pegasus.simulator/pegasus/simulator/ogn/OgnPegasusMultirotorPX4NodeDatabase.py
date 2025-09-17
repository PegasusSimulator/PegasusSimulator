"""Support for simplified access to data on nodes of type pegasus.simulator.PegasusMultirotorPX4Node

PX4-specific Pegasus Multirotor OmniGraph Node Database
Provides comprehensive interface for all PX4 backend configuration parameters.
"""

import carb
import sys
import traceback
from enum import IntEnum, auto

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn

import omni.ext
import omni.ui as ui
import omni.kit.commands

from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path


PX4_MULTIROTOR_FIELDS = [
    "EXEC_IN",
    "DRONE_PRIM",
    "VEHICLE_ID",
    "USD_FILE",
    "CONNECTION_TYPE",
    "CONNECTION_IP",
    "CONNECTION_BASEPORT",
    "PX4_AUTOLAUNCH",
    "PX4_DIR",
    "PX4_VEHICLE_MODEL",
    "ENABLE_LOCKSTEP",
    "NUM_ROTORS",
    "UPDATE_RATE",
    "INIT_POS_X",
    "INIT_POS_Y",
    "INIT_POS_Z",
    "INIT_ORIENT_X",
    "INIT_ORIENT_Y",
    "INIT_ORIENT_Z",
    "INIT_ORIENT_W",
    "INPUT_OFFSET_0",
    "INPUT_OFFSET_1",
    "INPUT_OFFSET_2",
    "INPUT_OFFSET_3",
    "INPUT_SCALING_0",
    "INPUT_SCALING_1",
    "INPUT_SCALING_2",
    "INPUT_SCALING_3",
    "ZERO_POSITION_ARMED_0",
    "ZERO_POSITION_ARMED_1",
    "ZERO_POSITION_ARMED_2",
    "ZERO_POSITION_ARMED_3",
]

# Enumeration for input indices dynamically generated from field names
PX4InputIndex = IntEnum(
    "PX4InputIndex",
    {name: i for i, name in enumerate(PX4_MULTIROTOR_FIELDS)}
)

class OgnPegasusMultirotorPX4NodeDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type pegasus.simulator.PegasusMultirotorPX4Node

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.execIn
            inputs.dronePrim
            inputs.vehicleID
            inputs.connectionType
            inputs.connectionIP
            inputs.connectionBaseport
            inputs.px4Autolaunch
            inputs.px4Dir
            inputs.px4VehicleModel
            inputs.enableLockstep
            inputs.numRotors
            inputs.inputOffset0-3
            inputs.inputScaling0-3
            inputs.zeroPositionArmed0-3
            inputs.updateRate
            inputs.initPosX
            inputs.initPosY
            inputs.initPosZ
            inputs.initOrientX
            inputs.initOrientY
            inputs.initOrientZ
            inputs.initOrientW
        Outputs:
            outputs.execOut
        State:
            state.omni_initialized
    """

    # Imprint the generator and target ABI versions in the file for JIT generation
    GENERATOR_VERSION = (1, 76, 0)
    TARGET_VERSION = (2, 170, 0)

    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}

    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface(
        [
            # Execution
            ("inputs:execIn", "execution", 0, None, "Signal to the graph that this node is ready to be executed.", {}, True, None, False, ""),
            ("outputs:execOut", "execution", 0, None, "Signal to the graph that execution can continue downstream.", {}, True, None, False, ""),
            
            # Basic vehicle configuration
            ("inputs:dronePrim", "string", 0, "Drone Prim", "Stage prefix for the drone vehicle.", {}, True, "/World/drone/Quadrotor", False, ""),
            ("inputs:vehicleID", "int", 0, "Vehicle ID", "Unique identifier for the vehicle.", {}, True, 0, False, ""),
            ("inputs:usdFile", "string", 0, "USD File", "Path to the USD file for the drone model.", {}, True, "/root/Documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd", False, ""),
            
            # Initial pose
            ("inputs:initPosX", "double", 0, "Initial Position X", "Initial X position of the vehicle.", {}, True, 0.0, False, ""),
            ("inputs:initPosY", "double", 0, "Initial Position Y", "Initial Y position of the vehicle.", {}, True, 0.0, False, ""),
            ("inputs:initPosZ", "double", 0, "Initial Position Z", "Initial Z position of the vehicle.", {}, True, 0.07, False, ""),
            ("inputs:initOrientX", "double", 0, "Initial Orientation X", "Initial X quaternion component.", {}, True, 0.0, False, ""),
            ("inputs:initOrientY", "double", 0, "Initial Orientation Y", "Initial Y quaternion component.", {}, True, 0.0, False, ""),
            ("inputs:initOrientZ", "double", 0, "Initial Orientation Z", "Initial Z quaternion component.", {}, True, 0.0, False, ""),
            ("inputs:initOrientW", "double", 0, "Initial Orientation W", "Initial W quaternion component.", {}, True, 1.0, False, ""),
            
            # Connection configuration
            ("inputs:connectionType", "string", 0, "Connection Type", "MAVLink connection type (e.g., 'tcpin', 'udp').", {}, True, "tcpin", False, ""),
            ("inputs:connectionIP", "string", 0, "Connection IP", "IP address for MAVLink connection.", {}, True, "localhost", False, ""),
            ("inputs:connectionBaseport", "int", 0, "Connection Base Port", "Base port for MAVLink connection.", {}, True, 4560, False, ""),
            
            # PX4-specific configuration
            ("inputs:px4Autolaunch", "bool", 0, "PX4 Autolaunch", "Whether to automatically launch PX4 in the background.", {}, True, True, False, ""),
            ("inputs:px4Dir", "string", 0, "PX4 Directory", "Path to PX4 installation directory.", {}, True, "/root/PX4-Autopilot", False, ""),
            ("inputs:px4VehicleModel", "string", 0, "PX4 Vehicle Model", "PX4 vehicle model name.", {}, True, "gazebo-classic_iris", False, ""),
            
            # Simulation configuration
            ("inputs:enableLockstep", "bool", 0, "Enable Lockstep", "Enable lockstep simulation mode.", {}, True, True, False, ""),
            ("inputs:numRotors", "int", 0, "Number of Rotors", "Number of rotors on the vehicle.", {}, True, 4, False, ""),
            ("inputs:updateRate", "double", 0, "Update Rate", "Backend update frequency in Hz.", {}, True, 250.0, False, ""),
            
            # Input configuration arrays (4 rotors)
            ("inputs:inputOffset0", "double", 0, "Input Offset 0", "Input offset for rotor 0.", {}, True, 0.0, False, ""),
            ("inputs:inputOffset1", "double", 0, "Input Offset 1", "Input offset for rotor 1.", {}, True, 0.0, False, ""),
            ("inputs:inputOffset2", "double", 0, "Input Offset 2", "Input offset for rotor 2.", {}, True, 0.0, False, ""),
            ("inputs:inputOffset3", "double", 0, "Input Offset 3", "Input offset for rotor 3.", {}, True, 0.0, False, ""),
            
            ("inputs:inputScaling0", "double", 0, "Input Scaling 0", "Input scaling factor for rotor 0.", {}, True, 1000.0, False, ""),
            ("inputs:inputScaling1", "double", 0, "Input Scaling 1", "Input scaling factor for rotor 1.", {}, True, 1000.0, False, ""),
            ("inputs:inputScaling2", "double", 0, "Input Scaling 2", "Input scaling factor for rotor 2.", {}, True, 1000.0, False, ""),
            ("inputs:inputScaling3", "double", 0, "Input Scaling 3", "Input scaling factor for rotor 3.", {}, True, 1000.0, False, ""),
            
            ("inputs:zeroPositionArmed0", "double", 0, "Zero Position Armed 0", "Zero position when armed for rotor 0.", {}, True, 100.0, False, ""),
            ("inputs:zeroPositionArmed1", "double", 0, "Zero Position Armed 1", "Zero position when armed for rotor 1.", {}, True, 100.0, False, ""),
            ("inputs:zeroPositionArmed2", "double", 0, "Zero Position Armed 2", "Zero position when armed for rotor 2.", {}, True, 100.0, False, ""),
            ("inputs:zeroPositionArmed3", "double", 0, "Zero Position Armed 3", "Zero position when armed for rotor 3.", {}, True, 100.0, False, ""),
            
            # State
            ("state:omni_initialized", "bool", 0, None, "State attribute used to control when the script should be reloaded.", {}, True, None, False, ""),
        ]
    )

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.outputs.execOut = og.AttributeRole.EXECUTION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {
            "execIn", "dronePrim", "vehicleID", "usdFile", "connectionType", "connectionIP", "connectionBaseport",
            "px4Autolaunch", "px4Dir", "px4VehicleModel", "enableLockstep", "numRotors", "updateRate",
            "initPosX", "initPosY", "initPosZ", "initOrientX", "initOrientY", "initOrientZ", "initOrientW",
            "inputOffset0", "inputOffset1", "inputOffset2", "inputOffset3",
            "inputScaling0", "inputScaling1", "inputScaling2", "inputScaling3",
            "zeroPositionArmed0", "zeroPositionArmed1", "zeroPositionArmed2", "zeroPositionArmed3",
            "_setting_locked", "_batchedReadAttributes", "_batchedReadValues",
        }
        """Helper class that creates natural hierarchical access to input attributes"""

        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [getattr(self._attributes, name) for name in [
                "execIn", "dronePrim", "vehicleID", "usdFile", "connectionType", "connectionIP", "connectionBaseport",
                "px4Autolaunch", "px4Dir", "px4VehicleModel", "enableLockstep", "numRotors", "updateRate",
                "initPosX", "initPosY", "initPosZ", "initOrientX", "initOrientY", "initOrientZ", "initOrientW",
                "inputOffset0", "inputOffset1", "inputOffset2", "inputOffset3",
                "inputScaling0", "inputScaling1", "inputScaling2", "inputScaling3",
                "zeroPositionArmed0", "zeroPositionArmed1", "zeroPositionArmed2", "zeroPositionArmed3"
            ]]
            self._batchedReadValues = [None] * len(self._batchedReadAttributes)

        # Property accessors for all inputs using enumeration indices
        @property
        def execIn(self): return self._batchedReadValues[PX4InputIndex.EXEC_IN]
        @execIn.setter
        def execIn(self, value): self._batchedReadValues[PX4InputIndex.EXEC_IN] = value

        @property
        def dronePrim(self): return self._batchedReadValues[PX4InputIndex.DRONE_PRIM]
        @dronePrim.setter
        def dronePrim(self, value): self._batchedReadValues[PX4InputIndex.DRONE_PRIM] = value

        @property
        def vehicleID(self): return self._batchedReadValues[PX4InputIndex.VEHICLE_ID]
        @vehicleID.setter
        def vehicleID(self, value): self._batchedReadValues[PX4InputIndex.VEHICLE_ID] = value

        @property
        def usdFile(self): return self._batchedReadValues[PX4InputIndex.USD_FILE]
        @usdFile.setter
        def usdFile(self, value): self._batchedReadValues[PX4InputIndex.USD_FILE] = value

        @property
        def connectionType(self): return self._batchedReadValues[PX4InputIndex.CONNECTION_TYPE]
        @connectionType.setter
        def connectionType(self, value): self._batchedReadValues[PX4InputIndex.CONNECTION_TYPE] = value

        @property
        def connectionIP(self): return self._batchedReadValues[PX4InputIndex.CONNECTION_IP]
        @connectionIP.setter
        def connectionIP(self, value): self._batchedReadValues[PX4InputIndex.CONNECTION_IP] = value

        @property
        def connectionBaseport(self): return self._batchedReadValues[PX4InputIndex.CONNECTION_BASEPORT]
        @connectionBaseport.setter
        def connectionBaseport(self, value): self._batchedReadValues[PX4InputIndex.CONNECTION_BASEPORT] = value

        @property
        def px4Autolaunch(self): return self._batchedReadValues[PX4InputIndex.PX4_AUTOLAUNCH]
        @px4Autolaunch.setter
        def px4Autolaunch(self, value): self._batchedReadValues[PX4InputIndex.PX4_AUTOLAUNCH] = value

        @property
        def px4Dir(self): return self._batchedReadValues[PX4InputIndex.PX4_DIR]
        @px4Dir.setter
        def px4Dir(self, value): self._batchedReadValues[PX4InputIndex.PX4_DIR] = value

        @property
        def px4VehicleModel(self): return self._batchedReadValues[PX4InputIndex.PX4_VEHICLE_MODEL]
        @px4VehicleModel.setter
        def px4VehicleModel(self, value): self._batchedReadValues[PX4InputIndex.PX4_VEHICLE_MODEL] = value

        @property
        def enableLockstep(self): return self._batchedReadValues[PX4InputIndex.ENABLE_LOCKSTEP]
        @enableLockstep.setter
        def enableLockstep(self, value): self._batchedReadValues[PX4InputIndex.ENABLE_LOCKSTEP] = value

        @property
        def numRotors(self): return self._batchedReadValues[PX4InputIndex.NUM_ROTORS]
        @numRotors.setter
        def numRotors(self, value): self._batchedReadValues[PX4InputIndex.NUM_ROTORS] = value

        @property
        def updateRate(self): return self._batchedReadValues[PX4InputIndex.UPDATE_RATE]
        @updateRate.setter
        def updateRate(self, value): self._batchedReadValues[PX4InputIndex.UPDATE_RATE] = value

        # Initial position properties
        @property
        def initPosX(self): return self._batchedReadValues[PX4InputIndex.INIT_POS_X]
        @initPosX.setter
        def initPosX(self, value): self._batchedReadValues[PX4InputIndex.INIT_POS_X] = value

        @property
        def initPosY(self): return self._batchedReadValues[PX4InputIndex.INIT_POS_Y]
        @initPosY.setter
        def initPosY(self, value): self._batchedReadValues[PX4InputIndex.INIT_POS_Y] = value

        @property
        def initPosZ(self): return self._batchedReadValues[PX4InputIndex.INIT_POS_Z]
        @initPosZ.setter
        def initPosZ(self, value): self._batchedReadValues[PX4InputIndex.INIT_POS_Z] = value

        # Initial orientation properties
        @property
        def initOrientX(self): return self._batchedReadValues[PX4InputIndex.INIT_ORIENT_X]
        @initOrientX.setter
        def initOrientX(self, value): self._batchedReadValues[PX4InputIndex.INIT_ORIENT_X] = value

        @property
        def initOrientY(self): return self._batchedReadValues[PX4InputIndex.INIT_ORIENT_Y]
        @initOrientY.setter
        def initOrientY(self, value): self._batchedReadValues[PX4InputIndex.INIT_ORIENT_Y] = value

        @property
        def initOrientZ(self): return self._batchedReadValues[PX4InputIndex.INIT_ORIENT_Z]
        @initOrientZ.setter
        def initOrientZ(self, value): self._batchedReadValues[PX4InputIndex.INIT_ORIENT_Z] = value

        @property
        def initOrientW(self): return self._batchedReadValues[PX4InputIndex.INIT_ORIENT_W]
        @initOrientW.setter
        def initOrientW(self, value): self._batchedReadValues[PX4InputIndex.INIT_ORIENT_W] = value

        # Input offset properties
        @property
        def inputOffset0(self): return self._batchedReadValues[PX4InputIndex.INPUT_OFFSET_0]
        @inputOffset0.setter
        def inputOffset0(self, value): self._batchedReadValues[PX4InputIndex.INPUT_OFFSET_0] = value

        @property
        def inputOffset1(self): return self._batchedReadValues[PX4InputIndex.INPUT_OFFSET_1]
        @inputOffset1.setter
        def inputOffset1(self, value): self._batchedReadValues[PX4InputIndex.INPUT_OFFSET_1] = value

        @property
        def inputOffset2(self): return self._batchedReadValues[PX4InputIndex.INPUT_OFFSET_2]
        @inputOffset2.setter
        def inputOffset2(self, value): self._batchedReadValues[PX4InputIndex.INPUT_OFFSET_2] = value

        @property
        def inputOffset3(self): return self._batchedReadValues[PX4InputIndex.INPUT_OFFSET_3]
        @inputOffset3.setter
        def inputOffset3(self, value): self._batchedReadValues[PX4InputIndex.INPUT_OFFSET_3] = value

        # Input scaling properties
        @property
        def inputScaling0(self): return self._batchedReadValues[PX4InputIndex.INPUT_SCALING_0]
        @inputScaling0.setter
        def inputScaling0(self, value): self._batchedReadValues[PX4InputIndex.INPUT_SCALING_0] = value

        @property
        def inputScaling1(self): return self._batchedReadValues[PX4InputIndex.INPUT_SCALING_1]
        @inputScaling1.setter
        def inputScaling1(self, value): self._batchedReadValues[PX4InputIndex.INPUT_SCALING_1] = value

        @property
        def inputScaling2(self): return self._batchedReadValues[PX4InputIndex.INPUT_SCALING_2]
        @inputScaling2.setter
        def inputScaling2(self, value): self._batchedReadValues[PX4InputIndex.INPUT_SCALING_2] = value

        @property
        def inputScaling3(self): return self._batchedReadValues[PX4InputIndex.INPUT_SCALING_3]
        @inputScaling3.setter
        def inputScaling3(self, value): self._batchedReadValues[PX4InputIndex.INPUT_SCALING_3] = value

        # Zero position armed properties
        @property
        def zeroPositionArmed0(self): return self._batchedReadValues[PX4InputIndex.ZERO_POSITION_ARMED_0]
        @zeroPositionArmed0.setter
        def zeroPositionArmed0(self, value): self._batchedReadValues[PX4InputIndex.ZERO_POSITION_ARMED_0] = value

        @property
        def zeroPositionArmed1(self): return self._batchedReadValues[PX4InputIndex.ZERO_POSITION_ARMED_1]
        @zeroPositionArmed1.setter
        def zeroPositionArmed1(self, value): self._batchedReadValues[PX4InputIndex.ZERO_POSITION_ARMED_1] = value

        @property
        def zeroPositionArmed2(self): return self._batchedReadValues[PX4InputIndex.ZERO_POSITION_ARMED_2]
        @zeroPositionArmed2.setter
        def zeroPositionArmed2(self, value): self._batchedReadValues[PX4InputIndex.ZERO_POSITION_ARMED_2] = value

        @property
        def zeroPositionArmed3(self): return self._batchedReadValues[PX4InputIndex.ZERO_POSITION_ARMED_3]
        @zeroPositionArmed3.setter
        def zeroPositionArmed3(self, value): self._batchedReadValues[PX4InputIndex.ZERO_POSITION_ARMED_3] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues

    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"execOut", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""

        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = {}

        @property
        def execOut(self):
            value = self._batchedWriteValues.get(self._attributes.execOut)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.execOut)
                return data_view.get()

        @execOut.setter
        def execOut(self, value):
            self._batchedWriteValues[self._attributes.execOut] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _commit(self):
            _og._commit_output_attributes_data(self._batchedWriteValues)
            self._batchedWriteValues = {}

    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""

        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)

        @property
        def omni_initialized(self):
            data_view = og.AttributeValueHelper(self._attributes.omni_initialized)
            return data_view.get()

        @omni_initialized.setter
        def omni_initialized(self, value):
            data_view = og.AttributeValueHelper(self._attributes.omni_initialized)
            data_view.set(value)

    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = OgnPegasusMultirotorPX4NodeDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnPegasusMultirotorPX4NodeDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnPegasusMultirotorPX4NodeDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnPegasusMultirotorPX4NodeDatabase.NODE_TYPE_CLASS, "get_node_type", None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return "pegasus.simulator.PegasusMultirotorPX4Node"

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            
            try:
                per_node_data = OgnPegasusMultirotorPX4NodeDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get("_db")
                if db is None:
                    db = OgnPegasusMultirotorPX4NodeDatabase(node)
                    per_node_data["_db"] = db
                if not database_valid():
                    per_node_data["_db"] = None
                    return False
            except Exception as e:
                db = OgnPegasusMultirotorPX4NodeDatabase(node)

            try:
                compute_function = getattr(OgnPegasusMultirotorPX4NodeDatabase.NODE_TYPE_CLASS, "compute", None)
                
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnPegasusMultirotorPX4NodeDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                print('Error in PX4 compute:', error)
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f"Assertion raised in compute - {error}\n{stack_trace}", add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnPegasusMultirotorPX4NodeDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnPegasusMultirotorPX4NodeDatabase.NODE_TYPE_CLASS, "initialize", None)
            if callable(initialize_function):
                initialize_function(context, node)

            per_node_data = OgnPegasusMultirotorPX4NodeDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data["_db"] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnPegasusMultirotorPX4NodeDatabase.NODE_TYPE_CLASS, "release", None)
            if callable(release_function):
                release_function(node)
            OgnPegasusMultirotorPX4NodeDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnPegasusMultirotorPX4NodeDatabase.NODE_TYPE_CLASS, "init_instance", None)
            if callable(init_instance_function):
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnPegasusMultirotorPX4NodeDatabase.NODE_TYPE_CLASS, "release_instance", None)
            if callable(release_instance_function):
                release_instance_function(node, graph_instance_id)
            OgnPegasusMultirotorPX4NodeDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnPegasusMultirotorPX4NodeDatabase.NODE_TYPE_CLASS, "update_node_version", None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnPegasusMultirotorPX4NodeDatabase.NODE_TYPE_CLASS, "initialize_type", None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "pegasus.simulator")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Pegasus PX4 Multirotor Node")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "pegasus")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "PX4-specific Pegasus multirotor simulation node.")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                icon_path = carb.tokens.get_tokens_interface().resolve("${pegasus.simulator}")
                icon_path = icon_path + "/" + "ogn/icons/pegasus.simulator.PegasusMultirotorPX4Node.svg"
                node_type.set_metadata(ogn.MetadataKeys.ICON_PATH, icon_path)
                __hints = node_type.get_scheduling_hints()
                if __hints is not None:
                    __hints.set_data_access(og.eAccessLocation.E_USD, og.eAccessType.E_WRITE)
                OgnPegasusMultirotorPX4NodeDatabase.INTERFACE.add_to_node_type(node_type)
                node_type.set_has_state(True)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnPegasusMultirotorPX4NodeDatabase.NODE_TYPE_CLASS, "on_connection_type_resolve", None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        print("PX4 node type class", node_type_class, type(node_type_class))
        OgnPegasusMultirotorPX4NodeDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnPegasusMultirotorPX4NodeDatabase.abi, 2)

    @staticmethod
    def deregister():
        og.deregister_node_type("pegasus.simulator.PegasusMultirotorPX4Node")

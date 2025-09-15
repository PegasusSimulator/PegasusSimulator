import omni.graph.core as og

class PegasusMultirotorNode:
    """Minimal logic class for Pegasus Multirotor node"""

    @staticmethod
    def compute(db):
        # Minimal compute: just enable execOut
        print("[DEBUG] PegasusMultirotorNode.compute called")
        if hasattr(db.outputs, 'execOut'):
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True


class PegasusMultirotorNodeDatabase(og.Database):
    """Database wrapper for PegasusMultirotorNode"""

    INTERFACE = og.Database._get_interface([
        ("inputs:execIn", "execution", 0, None, "Trigger execution", {}, True, None, False, ""),
        ("outputs:execOut", "execution", 0, None, "Execution output", {}, True, None, False, ""),
    ])

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"execIn", "_batchedReadValues"}

        def __init__(self, node, attributes, dynamic_attributes):
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadValues = [None]

        @property
        def execIn(self):
            return self._batchedReadValues[0]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[0] = value

        def _prefetch(self):
            pass

    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"execOut", "_batchedWriteValues"}

        def __init__(self, node, attributes, dynamic_attributes):
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = {}

        @property
        def execOut(self):
            return self._batchedWriteValues.get(self._attributes.execOut, 0)

        @execOut.setter
        def execOut(self, value):
            self._batchedWriteValues[self._attributes.execOut] = value

        def _commit(self):
            pass

    def __init__(self, node):
        super().__init__(node)
        dyn_inputs = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        dyn_outputs = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.inputs = PegasusMultirotorNodeDatabase.ValuesForInputs(node, self.attributes.inputs, dyn_inputs)
        self.outputs = PegasusMultirotorNodeDatabase.ValuesForOutputs(node, self.attributes.outputs, dyn_outputs)

    class abi:
        @staticmethod
        def get_node_type():
            return "pegasus.simulator.PegasusMultirotorNode"

        @staticmethod
        def compute(context, node):
            try:
                db = PegasusMultirotorNodeDatabase(node)
                return PegasusMultirotorNode.compute(db)
            except Exception as e:
                print(f"[ERROR] PegasusMultirotorNode compute failed: {e}")
                return False

    @staticmethod
    def register():
        """Register the node with OmniGraph"""
        try:
            og.register_node_type(PegasusMultirotorNodeDatabase.abi, 2)
            print("=" * 60)
            print("[DEBUG] PegasusMultirotorNode registered successfully")
            print("=" * 60)
        except Exception as e:
            print(f"[ERROR] Failed to register PegasusMultirotorNode: {e}")


# Automatically register on import
PegasusMultirotorNodeDatabase.register()



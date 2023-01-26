#!/usr/bin/env python

# Sensors that can be used with the vehicles
from pegasus_isaac.parser import Parser
from pegasus_isaac.logic.dynamics import LinearDrag

class DynamicsParser(Parser):

    def __init__(self):

        # Dictionary of available sensors to instantiate
        self.dynamics = {
            "linear_drag": LinearDrag
        }

    def parse(self, data_type: str, data_dict):
        
        # Get the class of the sensor
        dynamics_cls = self.dynamics[data_type]

        # Create an instance of that sensor
        return dynamics_cls(data_dict)

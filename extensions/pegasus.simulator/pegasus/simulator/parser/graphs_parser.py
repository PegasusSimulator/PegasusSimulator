# Copyright (c) 2023, Marcelo Jacinto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Graphs that can be used with the vehicles
from pegasus.simulator.parser import Parser
from pegasus.simulator.logic.graphs import (
    ROS2Camera,
    ROS1Camera,
    ROS1Clock,
    ROS1TF,
    ROS1Odom,
)


class GraphParser(Parser):
    def __init__(self):

        # Dictionary of available graphs to instantiate
        self.graphs = {
            "ROS2 Camera": ROS2Camera,
            "ROS1 Camera": ROS1Camera,
            "ROS1 Clock": ROS1Clock,
            "ROS1 TF": ROS1TF,
            "ROS1 Odom": ROS1Odom,
        }

    def parse(self, data_type: str, data_dict):

        # Get the class of the graph
        graph_cls = self.graphs[data_type]

        # Create an instance of that graph
        return graph_cls(data_dict)

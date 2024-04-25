"""
| File: ros2_clock.py
| License: BSD-3-Clause. Copyright (c) 2023, Micah Nye. All rights reserved.
"""
__all__ = ["ROS2Clock"]

import carb

import omni.graph.core as og
from omni.isaac.core.utils.prims import is_prim_path_valid

from pegasus.simulator.logic.graphs import Graph
from pegasus.simulator.logic.vehicles import Vehicle


class ROS2Clock(Graph):
    """The class that implements the ROS2 Clock graph. This class inherits the base class Graph."""

    def __init__(self, config: dict = {}):
        """Initialize the ROS2 Clock class

        Examples:
            The dictionary default parameters are

            >>> {"reset_on_stop": False}                # if true, the simulation time will reset when stop is pressed
        """

        # Initialize the Super class "object" attribute
        super().__init__(graph_type="ROS2Clock")

        # Process the config dictionary
        self._reset_on_stop = config.get("reset_on_stop", False)

    def initialize(self, vehicle: Vehicle):
        """Method that initializes the graph.

        Args:
            vehicle (Vehicle): The vehicle that this graph is attached to.
        """

        # Create the graph under world with graph name clock and allow only one per world.
        graph_path = "/World/clock_pub"
        if is_prim_path_valid(graph_path):
            carb.log_warn(f'{"ROS2 Clock Graph already exists"}')
            return

        # Graph configuration
        graph_specs = {
            "graph_path": graph_path,
            "evaluator_name": "execution",
        }

        # Creating a graph edit configuration
        keys = og.Controller.Keys
        graph_config = {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnPlaybackTick"),
                ("simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("publish_clock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "publish_clock.inputs:execIn"),
                ("simulation_time.outputs:simulationTime", "publish_clock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("simulation_time.inputs:resetOnStop", self._reset_on_stop),
                ("publish_clock.inputs:topicName", "/clock"),
            ],
        }

        # Create the clock graph
        (graph, _, _, _) = og.Controller.edit(graph_specs, graph_config)

        # Run the clock graph once to generate ROS clock publisher in SDGPipeline
        og.Controller.evaluate_sync(graph)

        # Also initialize the Super class with updated prim path
        super().initialize(graph_path)

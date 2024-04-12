"""
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto and Filip Stec. All rights reserved.
"""
__all__ = ["Graph"]

class Graph:
    """The base class for implementing OmniGraphs

    Attributes:
        graph_prim_path
    """
    def __init__(self, graph_type: str):
        """Initialize Graph class

        Args:
            graph_type (str): A name that describes the type of graph
        """
        self._graph_type = graph_type
        self._graph_prim_path = None

    def initialize(self, graph_prim_path: str):
        """
        Method that should be implemented and called by the class that inherits the graph object.
        """
        self._graph_prim_path = graph_prim_path

    @property
    def graph_type(self) -> str:
        """
        (str) A name that describes the type of graph.
        """
        return self._graph_type

    @property
    def graph_prim_path(self) -> str:
        """
        (str) Path to the graph.
        """
        return self._graph_prim_path

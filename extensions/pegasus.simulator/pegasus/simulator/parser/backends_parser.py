# Copyright (c) 2023, Marcelo Jacinto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Sensors that can be used with the vehicles
from pegasus.simulator.parser import Parser
from pegasus.simulator.logic.backends import MavlinkBackendConfig, MavlinkBackend, ROS2Backend


class BackendsParser(Parser):
    # TODO - improve the structure of the backends in order to clean this parser

    def __init__(self):

        # Dictionary of available sensors to instantiate
        self.backends = {"mavlink": MavlinkBackendConfig, "ros2": ROS2Backend}

    def parse(self, data_type: str, data_dict):

        # Get the class of the sensor
        backends_cls = self.backends[data_type]

        if backends_cls == MavlinkBackendConfig:
            return MavlinkBackend(backends_cls(data_dict))

        # Create an instance of that sensor
        return backends_cls(data_dict)

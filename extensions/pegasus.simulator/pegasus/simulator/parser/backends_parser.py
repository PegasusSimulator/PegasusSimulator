# Copyright (c) 2023, Marcelo Jacinto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Sensors that can be used with the vehicles
from pegasus.simulator.parser import Parser
from pegasus.simulator.logic.backends import PX4MavlinkBackend, ArduPilotMavlinkBackend, ROS2Backend
from pegasus.simulator.params import BACKENDS

class BackendsParser(Parser):
    # TODO - improve the structure of the backends in order to clean this parser

    ##############################
    ############ TODO - Redundancy ##############
    ############# TODO TODO TODO TODO ##############

    def __init__(self):

        # Dictionary of available sensors to instantiate
        self.backends = {
            BACKENDS["px4"]: PX4MavlinkBackend,
            BACKENDS["ardupilot"]: ArduPilotMavlinkBackend,
            BACKENDS["ros2"]: ROS2Backend
        }

    def parse(self, data_type: str, data_dict):

        # Get the class of the sensor
        backends_cls = self.backends[data_type]

        if backends_cls == PX4MavlinkBackend:
            return PX4MavlinkBackend(backends_cls(data_dict))
        elif backends_cls == ArduPilotMavlinkBackend:
            return ArduPilotMavlinkBackend(backends_cls(data_dict))
        elif backends_cls == ROS2Backend:
            return ROS2Backend(backends_cls(data_dict))

        # Create an instance of that sensor
        return backends_cls(data_dict)

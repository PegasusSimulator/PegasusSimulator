# Copyright (c) 2023, Marcelo Jacinto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Sensors that can be used with the vehicles
from pegasus.simulator.parser import Parser
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve


class ThrustersParser(Parser):
    def __init__(self):

        # Dictionary of available thrust curves to instantiate
        self.thrust_curves = {"quadratic_thrust_curve": QuadraticThrustCurve}

    def parse(self, data_type: str, data_dict):

        # Get the class of the sensor
        thrust_curve_cls = self.thrust_curves[data_type]

        # Create an instance of that sensor
        return thrust_curve_cls(data_dict)

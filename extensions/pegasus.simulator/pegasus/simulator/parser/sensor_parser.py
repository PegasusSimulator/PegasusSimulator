# Copyright (c) 2023, Marcelo Jacinto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Sensors that can be used with the vehicles
from pegasus.simulator.parser import Parser
from pegasus.simulator.logic.sensors import Barometer, GPS, IMU, Magnetometer


class SensorParser(Parser):
    def __init__(self):

        # Dictionary of available sensors to instantiate
        self.sensors = {"barometer": Barometer, "gps": GPS, "imu": IMU, "magnetometer": Magnetometer}

    def parse(self, data_type: str, data_dict):

        # Get the class of the sensor
        sensor_cls = self.sensors[data_type]

        # Create an instance of that sensor
        return sensor_cls(data_dict)

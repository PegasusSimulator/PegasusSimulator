# Copyright (c) 2023, Marcelo Jacinto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import carb

# Sensors that can be used with the vehicles
from pegasus.simulator.parser import Parser, SensorParser, ThrustersParser, DynamicsParser, BackendsParser
from pegasus.simulator.logic.vehicles import MultirotorConfig


class VehicleParser(Parser):
    def __init__(self):

        # Initialize the Parser object
        super().__init__()

        # Initialize Parsers for the sensors, dynamics and backends for control and communications
        self.sensor_parser = SensorParser()
        self.thrusters_parser = ThrustersParser()
        self.dynamics_parser = DynamicsParser()
        self.backends_parser = BackendsParser()

    def parse(self, data_type: str, data_dict={}):

        # Get the USD model associated with the vehicle
        usd_model = data_dict.get("usd_model", "")

        # Get the model thumbnail of the vehicle
        thumbnail = data_dict.get("thumbnail", "")

        # ---------------------------------------
        # Generate the sensors for the multirotor
        # ---------------------------------------
        sensors = []
        sensors_config = data_dict.get("sensors", {})

        for sensor_name in sensors_config:
            sensor = self.sensor_parser.parse(sensor_name, sensors_config[sensor_name])
            if sensor is not None:
                sensors.append(sensor)

        # -----------------------------------------
        # Generate the thrusters for the multirotor
        # -----------------------------------------
        thrusters = None
        thrusters_config = data_dict.get("thrusters", {})

        # Note: if a dictionary/yaml file contains more than one thrust curve configuration,
        # only the last one will be kept
        for thrust_curve_name in thrusters_config:
            curve = self.thrusters_parser.parse(thrust_curve_name, thrusters_config[thrust_curve_name])
            if curve is not None:
                thrusters = curve

        # ----------------------------------------
        # Generate the dynamics for the multirotor
        # ----------------------------------------
        dynamics = None
        dynamics_config = data_dict.get("drag", {})

        for dynamics_name in dynamics_config:
            carb.log_warn(dynamics_config[dynamics_name])
            dynamic = self.dynamics_parser.parse(dynamics_name, dynamics_config[dynamics_name])
            if dynamic is not None:
                dynamics = dynamic

        # ----------------------------------------
        # Generate the backends for the multirotor
        # ----------------------------------------
        backends = []
        backends_config = data_dict.get("backends", {})

        for backends_name in backends_config:
            backend = self.backends_parser.parse(backends_name, backends_config[backends_name])
            if backend is not None:
                backends.append(backend)

        # Create a Multirotor config from the parsed data
        multirotor_configuration = MultirotorConfig()
        multirotor_configuration.usd_file = usd_model
        multirotor_configuration.thrust_curve = thrusters
        multirotor_configuration.drag = dynamics
        multirotor_configuration.sensors = sensors
        multirotor_configuration.backends = backends

        return multirotor_configuration

# Copyright (c) 2023, Marcelo Jacinto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig

# Sensors and dynamics setup
from pegasus.simulator.logic.dynamics import LinearDrag
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS

# Mavlink interface
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend

# Get the location of the IRIS asset
from pegasus.simulator.params import ROBOTS

class IrisConfig(MultirotorConfig):

    def __init__(self):

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "quadrotor"

        # The USD file that describes the visual aspect of the vehicle (and some properties such as mass and moments of inertia)
        self.usd_file = ROBOTS["Iris"]

        # The default thrust curve for a quadrotor and dynamics relating to drag
        self.thrust_curve = QuadraticThrustCurve()
        self.drag = LinearDrag([0.50, 0.30, 0.0])

        # The default sensors for a quadrotor
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]

        # The backends for actually sending commands to the vehicle. By default use mavlink (with default mavlink configurations)
        # [Can be None as well, if we do not desired to use PX4 with this simulated vehicle]. It can also be a ROS2 backend
        # or your own custom Backend implementation!
        self.backends = [MavlinkBackend()]

class Iris(Multirotor):

    def __init__(self, id: int, world, init_pos=[0.0, 0.0, 0.07, init_orientation=[0.0, 0.0, 0.0, 1.0]], config=IrisConfig()):
        super.__init__(config.stage_prefix, config.usd_file, id, world, init_pos, init_orientation, config=config)
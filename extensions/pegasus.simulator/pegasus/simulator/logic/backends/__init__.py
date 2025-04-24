"""
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""

from .backend import Backend, BackendConfig
from .px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from .ardupilot_mavlink_backend import ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig

# Check if the ROS2 package is installed
try:
    from .ros2_backend import ROS2Backend
except:
    import carb
    carb.log_warn("ROS2 package not installed. ROS2Backend will not be available")
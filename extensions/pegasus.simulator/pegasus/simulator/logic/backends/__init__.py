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
except Exception as e:
    import carb
    import traceback
    carb.log_warn(f"ROS2Backend not available: {e}\n{traceback.format_exc()}")
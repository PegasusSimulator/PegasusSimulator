"""
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
"""

from .people_backend import PeopleBackend

# Check if the ROS2 package is installed
try:
    from .ros2_people_backend_backend import ROS2PeopleBackend
except:
    import carb
    carb.log_warn("ROS2 package not installed. ROS2PeopleBackend will not be available")
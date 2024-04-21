import carb

import omni.kit.app


def get_ros_extension():
    """
    Method that checks which ROS extension is installed.

    Returns:
        str: The version of the ROS extension installed. Possible values are "ros", "ros2", or an empty string if neither extension is enabled.
    """

    # Get the handle for the extension manager
    extension_manager = omni.kit.app.get_app().get_extension_manager()

    version = ""

    if extension_manager.is_extension_enabled("omni.isaac.ros_bridge"):
        version = "ros"
    elif extension_manager.is_extension_enabled("omni.isaac.ros2_bridge"):
        version = "ros2"
    else:
        carb.log_warn("Neither extension 'omni.isaac.ros_bridge' nor 'omni.isaac.ros2_bridge' is enabled")
    carb.log_info(f"ROS extension version: {version}")
    return version

"""
| File: ros2_backend.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: File that implements the ROS2 Backend for communication/control with/of the vehicle simulation through ROS2 topics
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""
import carb
from omni.isaac.core.utils.extensions import disable_extension, enable_extension

import omni.kit.app
from pegasus.simulator.logic.backends.backend import Backend


class ROS2BackendConfig:
    def __init__(self):
        pass


class ROS2Backend(Backend):

    def __init__(self, vehicle_id: int):

        # Save the configurations for this backend
        self._id = vehicle_id
        
        # Perform some checks, because Isaac Sim some times does not play nice when using ROS/ROS2
        #disable_extension("omni.isaac.ros_bridge")
        #disable_extension("omni.isaac.ros2_bridge")
        #enable_extension("omni.isaac.ros2_bridge-humble")
        
        # Inform the user that now we are actually import the ROS2 dependencies 
        import rclpy
        from sensor_msgs.msg import Imu, NavSatFix
        from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
        carb.log_warn("rclpy was imported successfully")

        # Start the actual ROS2 setup here
        rclpy.init()
        self.node = rclpy.create_node("vehicle_1")

        # Create publishers for the state of the vehicle in ENU
        self.pose_pub = self.node.create_publisher(PoseStamped, "vehicle" + str(self._id) + "/state/pose", 10)
        self.twist_pub = self.node.create_publisher(TwistStamped, "vehicle" + str(self._id) + "/state/twist", 10)
        self.accel_pub = self.node.create_publisher(AccelStamped, "vehicle" + str(self._id) + "/state/accel", 10)

        # Create publishers for some sensor data
        self.imu_pub = self.node.create_publisher(Imu, "vehicle" + str(self._id) + "/sensors/imu", 10)
        self.gps_pub = self.node.create_publisher(NavSatFix, "vehicle" + str(self._id) + "/sensors/gps", 10)

        # Subscribe to vector of floats with the target angular velocities to control the vehicle
        #self.vel_sub = self.node.create_subscription(LaserScan, "scan", lidar_callback, 10)
        

    def update_sensor(self, sensor_type: str, data):
        """
        Method that when implemented, should handle the receival of sensor data
        """
        pass

    def update_state(self, state):
        """
        Method that when implemented, should handle the receivel of the state of the vehicle using this callback
        """
        pass

    def input_reference(self):
        """
        Method that when implemented, should return a list of desired angular velocities to apply to the vehicle rotors
        """
        return []

    def update(self, dt: float):
        """
        Method that when implemented, should be used to update the state of the backend and the information being sent/received
        from the communication interface. This method will be called by the simulation on every physics step
        """
        pass

    def start(self):
        """
        Method that when implemented should handle the begining of the simulation of vehicle
        """
        pass

    def stop(self):
        """
        Method that when implemented should handle the stopping of the simulation of vehicle
        """
        pass

    def reset(self):
        """
        Method that when implemented, should handle the reset of the vehicle simulation to its original state
        """
        pass

    def check_ros_extension(self):
        """
        Method that checks which ROS extension is installed.
        """

        # Get the handle for the extension manager
        extension_manager = omni.kit.app.get_app().get_extension_manager()

        version = ""

        if self._ext_manager.is_extension_enabled("omni.isaac.ros_bridge"):
            version = "ros"
        elif self._ext_manager.is_extension_enabled("omni.isaac.ros2_bridge"):
            version = "ros2"
        else:
            carb.log_warn("Neither extension 'omni.isaac.ros_bridge' nor 'omni.isaac.ros2_bridge' is enabled")

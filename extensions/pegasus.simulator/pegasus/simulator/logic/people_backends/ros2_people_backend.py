"""
| File: ros2_people_backend.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: File that implements the ROS2 Backend for publishing a person state to ROS 2
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
"""

__all__ = ["ROS2PeopleBackend"]

# Make sure the ROS2 extension is enabled
import carb
from pegasus.simulator.logic.people_backends.people_backend import PeopleBackend
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

# ROS2 imports
import rclpy
from geometry_msgs.msg import PoseStamped

class ROS2PeopleBackend(PeopleBackend):

    def __init__(self, person_id: int, namespace="people", topic="person"):
        """
        Initialize the ROS2 People Backend                
        """

        # Save the configurations for this backend
        self._id = person_id
        self._namespace = namespace
        self._topic = topic

        # Start the actual ROS2 setup here
        try:
            rclpy.init()
        except:
            # If rclpy is already initialized, just ignore the exception
            pass

        self.node = rclpy.create_node("simulator_people_" + str(person_id))

        # Initialize the publishers and subscribers
        self.initialize_publishers()
    
    
    def initialize_publishers(self):

        # ----------------------------------------------------- 
        # Create publishers for the state of the person in ENU
        # -----------------------------------------------------
        self.pose_pub = self.node.create_publisher(PoseStamped, self._namespace + "/" + self._topic + str(self._id), rclpy.qos.qos_profile_sensor_data)
    

    def update_state(self, state):
        """
        Method that when implemented, should handle the receivel of the state of the vehicle using this callback
        """

        # Publish the state of the vehicle only if the flag is set to True
        pose = PoseStamped()

        # Update the header
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        # Fill the position and attitude of the vehicle in ENU
        pose.pose.position.x = state.position[0]
        pose.pose.position.y = state.position[1]
        pose.pose.position.z = state.position[2] + 1.0 # Add 1.0 to the z position to simulate the middle of the height of the person

        pose.pose.orientation.x = state.attitude[0]
        pose.pose.orientation.y = state.attitude[1]
        pose.pose.orientation.z = state.attitude[2]
        pose.pose.orientation.w = state.attitude[3]

        # Publish the messages containing the state of the vehicle
        self.pose_pub.publish(pose)


    def update(self, state, dt: float):
        """
        Method that when implemented, should be used to update the state of the backend and the information being sent/received
        from the communication interface. This method will be called by the simulation on every physics step
        """

        # Update the state of the person
        self.update_state(state)

        # In this case, do nothing as we are sending messages as soon as new data arrives from the sensors and state
        # and updating the reference for the thrusters as soon as receiving from ROS2 topics
        # Just poll for new ROS 2 messages in a non-blocking way
        rclpy.spin_once(self.node, timeout_sec=0)
"""
| File: ros2_backend.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: File that implements the ROS2 Backend for communication/control with/of the vehicle simulation through ROS2 topics
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""
import os
from omni.isaac.core.utils.extensions import disable_extension, enable_extension

# Get the environment variable for the ROS2 distribution
ROS_DISTRO = os.getenv("ROS_DISTRO", "foxy")

# Enable the correct ROS2 extension
if ROS_DISTRO == "foxy" or ROS_DISTRO == "humble":
    
    # Ensure that the ROS1 extension is disabled and we are enabling the ROS2 extension
    disable_extension("omni.isaac.ros_bridge")
    disable_extension("omni.isaac.ros2_bridge-humble")
    enable_extension("omni.isaac.ros2_bridge")

# TODO - ROS2 humble provided by nvidia is not working properly. For some reason we can also use the foxy extension
# even when using ROS2 humble on ubuntu 22.04LTS. This is a temporary fix until NVIDIA figures this thing out (hopefully)
# elif ROS_DISTRO == "humble":
#     disable_extension("omni.isaac.ros2_bridge")
#     enable_extension("omni.isaac.ros2_bridge-humble")

import rclpy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus, Image
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped

from pegasus.simulator.logic.backends.backend import Backend

class ROS2Backend(Backend):

    def __init__(self, vehicle_id: int, num_rotors=4):

        # Save the configurations for this backend
        self._id = vehicle_id
        self._num_rotors = num_rotors

        # Start the actual ROS2 setup here
        rclpy.init()
        self.node = rclpy.create_node("vehicle_" + str(vehicle_id))

        # Create publishers for the state of the vehicle in ENU
        self.pose_pub = self.node.create_publisher(PoseStamped, "vehicle" + str(self._id) + "/state/pose", 10)
        self.twist_pub = self.node.create_publisher(TwistStamped, "vehicle" + str(self._id) + "/state/twist", 10)
        self.twist_inertial_pub = self.node.create_publisher(TwistStamped, "vehicle" + str(self._id) + "/state/twist_inertial", 10)
        self.accel_pub = self.node.create_publisher(AccelStamped, "vehicle" + str(self._id) + "/state/accel", 10)

        # Create publishers for some sensor data
        self.imu_pub = self.node.create_publisher(Imu, "vehicle" + str(self._id) + "/sensors/imu", 10)
        self.mag_pub = self.node.create_publisher(MagneticField, "vehicle" + str(self._id) + "/sensors/mag", 10)
        self.gps_pub = self.node.create_publisher(NavSatFix, "vehicle" + str(self._id) + "/sensors/gps", 10)
        self.gps_vel_pub = self.node.create_publisher(TwistStamped, "vehicle" + str(self._id) + "/sensors/gps_twist", 10)

        # Create publishers for the camera images
        self.cam_pubs = {}

        # Subscribe to vector of floats with the target angular velocities to control the vehicle
        # This is not ideal, but we need to reach out to NVIDIA so that they can improve the ROS2 support with custom messages
        # The current setup as it is.... its a pain!!!!
        self.rotor_subs = []
        for i in range(self._num_rotors):
            self.rotor_subs.append(self.node.create_subscription(Float64, "vehicle" + str(self._id) + "/control/rotor" + str(i) + "/ref", lambda x: self.rotor_callback(x, i),10))
    
        # Setup zero input reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]


    def update_state(self, state):
        """
        Method that when implemented, should handle the receivel of the state of the vehicle using this callback
        """

        pose = PoseStamped()
        twist = TwistStamped()
        twist_inertial = TwistStamped()
        accel = AccelStamped()

        # Update the header
        pose.header.stamp = self.node.get_clock().now().to_msg()
        twist.header.stamp = pose.header.stamp
        twist_inertial.header.stamp = pose.header.stamp
        accel.header.stamp = pose.header.stamp

        pose.header.frame_id = "world"
        twist.header.frame_id = "base_link"
        twist_inertial.header.frame_id = "world"
        accel.header.frame_id = "world"

        # Fill the position and attitude of the vehicle in ENU
        pose.pose.position.x = state.position[0]
        pose.pose.position.y = state.position[1]
        pose.pose.position.z = state.position[2]

        pose.pose.orientation.x = state.attitude[0]
        pose.pose.orientation.y = state.attitude[1]
        pose.pose.orientation.z = state.attitude[2]
        pose.pose.orientation.w = state.attitude[3]

        # Fill the linear and angular velocities in the body frame of the vehicle
        twist.twist.linear.x = state.linear_body_velocity[0]
        twist.twist.linear.y = state.linear_body_velocity[1]
        twist.twist.linear.z = state.linear_body_velocity[2]

        twist.twist.angular.x = state.angular_velocity[0]
        twist.twist.angular.y = state.angular_velocity[1]
        twist.twist.angular.z = state.angular_velocity[2]

        # Fill the linear velocity of the vehicle in the inertial frame
        twist_inertial.twist.linear.x = state.linear_velocity[0]
        twist_inertial.twist.linear.y = state.linear_velocity[1]
        twist_inertial.twist.linear.z = state.linear_velocity[2]

        # Fill the linear acceleration in the inertial frame
        accel.accel.linear.x = state.linear_acceleration[0]
        accel.accel.linear.y = state.linear_acceleration[1]
        accel.accel.linear.z = state.linear_acceleration[2]

        # Publish the messages containing the state of the vehicle
        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.twist_inertial_pub.publish(twist_inertial)
        self.accel_pub.publish(accel)

    def rotor_callback(self, ros_msg, rotor_id):
        # Update the reference for the rotor of the vehicle
        self.input_ref[rotor_id] = float(ros_msg.data)

    def update_sensor(self, sensor_type: str, data):
        """
        Method that when implemented, should handle the receival of sensor data
        """

        if sensor_type == "IMU":
            self.update_imu_data(data)
        elif sensor_type == "GPS":
            self.update_gps_data(data)
        elif sensor_type == "Magnetometer":
            self.update_mag_data(data)
        elif sensor_type == "Barometer":        
            pass    # TODO - create a topic for the barometer later on
        elif sensor_type == "RGBCamera":
            self.update_camera_data(data)

    def update_camera_data(self, data):
        """
        Method used to update the data received by the RGB camera. Note: since
        encoding the image to bytes is a costly operation, we will we will only
        do this step in another thread, and we will only publish the image
        message in this thread after the encoding is done.

        Args:
            data (dict): Dictionary with the frame and its metadata
        """

        # Check if we already have a publisher for that camera id. If not, create one
        if data["id"] not in self.cam_pubs:
            self.cam_pubs[data["id"]] = self.node.create_publisher(Image, "vehicle" + str(self._id) + "/camera" + str(data["id"]) + "/image_raw", 10)

        # Create an empty image message
        image_msg = Image()

        # Check if the camera is already registered (the first frames are usually empty)
        if data["frame"].shape[0] > 0 and data["frame"].shape[1] > 0:

            # Fill the image message
            image_msg.header.stamp = self.node.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera" + str(data["id"])
            image_msg.height = data["frame"].shape[0]
            image_msg.width = data["frame"].shape[1]
            image_msg.step = image_msg.width * 3

            # Note: Assign the _data object directly. 
            # This is a workaround, because the assignment of the data object is painfully slow
            image_msg._data = data["frame"][:, :, 0:3].tobytes()

            image_msg.encoding = "rgb8"
            image_msg.is_bigendian = 0

        # Publish the image message
        self.cam_pubs[data["id"]].publish(image_msg)

    def update_imu_data(self, data):

        msg = Imu()

        # Update the header
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link_frd"
        
        # Update the angular velocity (NED + FRD)
        msg.angular_velocity.x = data["angular_velocity"][0]
        msg.angular_velocity.y = data["angular_velocity"][1]
        msg.angular_velocity.z = data["angular_velocity"][2]
        
        # Update the linear acceleration (NED)
        msg.linear_acceleration.x = data["linear_acceleration"][0]
        msg.linear_acceleration.y = data["linear_acceleration"][1]
        msg.linear_acceleration.z = data["linear_acceleration"][2]

        # Publish the message with the current imu state
        self.imu_pub.publish(msg)

    def update_gps_data(self, data):

        msg = NavSatFix()
        msg_vel = TwistStamped()

        # Update the headers
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "world_ned"
        msg_vel.header.stamp = msg.header.stamp
        msg_vel.header.frame_id = msg.header.frame_id

        # Update the status of the GPS
        status_msg = NavSatStatus()
        status_msg.status = 0 # unaugmented fix position
        status_msg.service = 1 # GPS service
        msg.status = status_msg

        # Update the latitude, longitude and altitude
        msg.latitude = data["latitude"]
        msg.longitude = data["longitude"]
        msg.altitude = data["altitude"]

        # Update the velocity of the vehicle measured by the GPS in the inertial frame (NED)
        msg_vel.twist.linear.x = data["velocity_north"]
        msg_vel.twist.linear.y = data["velocity_east"]
        msg_vel.twist.linear.z = data["velocity_down"]

        # Publish the message with the current GPS state
        self.gps_pub.publish(msg)
        self.gps_vel_pub.publish(msg_vel)

    def update_mag_data(self, data):
        
        msg = MagneticField()

        # Update the headers
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link_frd"

        msg.magnetic_field.x = data["magnetic_field"][0]
        msg.magnetic_field.y = data["magnetic_field"][1]
        msg.magnetic_field.z = data["magnetic_field"][2]

        # Publish the message with the current magnetic data
        #self.mag_pub.publish(msg)

    def input_reference(self):
        """
        Method that is used to return the latest target angular velocities to be applied to the vehicle

        Returns:
            A list with the target angular velocities for each individual rotor of the vehicle
        """
        return self.input_ref

    def update(self, dt: float):
        """
        Method that when implemented, should be used to update the state of the backend and the information being sent/received
        from the communication interface. This method will be called by the simulation on every physics step
        """

        # In this case, do nothing as we are sending messages as soon as new data arrives from the sensors and state
        # and updating the reference for the thrusters as soon as receiving from ROS2 topics
        # Just poll for new ROS 2 messages in a non-blocking way
        rclpy.spin_once(self.node, timeout_sec=0)

    def start(self):
        """
        Method that when implemented should handle the begining of the simulation of vehicle
        """
        # Reset the reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]

    def stop(self):
        """
        Method that when implemented should handle the stopping of the simulation of vehicle
        """
        # Reset the reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]

    def reset(self):
        """
        Method that when implemented, should handle the reset of the vehicle simulation to its original state
        """
        # Reset the reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]

"""
| File: ros2_backend.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: File that implements the ROS2 Backend for communication/control with/of the vehicle simulation through ROS2 topics
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
"""

# Make sure the ROS2 extension is enabled
import carb
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

# ROS2 imports
import rclpy
from rclpy.time import Time
from rclpy.clock import ClockType
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped

# TF imports
# Check if these libraries exist in the system
try:
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from tf2_ros.transform_broadcaster import TransformBroadcaster
    tf2_ros_loaded = True
except ImportError:
    carb.log_warn("TF2 ROS not installed. Will not publish TFs with the ROS2 backend")
    tf2_ros_loaded = False

from pegasus.simulator.logic.backends.backend import Backend

# Import the replicatore core module used for writing graphical data to ROS 2
import omni
import omni.graph.core as og
import omni.replicator.core as rep
from isaacsim.ros2.bridge import read_camera_info


class ROS2Backend(Backend):

    # Class-level flag to track if clock publisher has been created by any instance
    _clock_publisher_created = False

    def __init__(self, vehicle_id: int, num_rotors=4, config: dict = {}):
        """Initialize the ROS2 Camera class

        Args:
            camera_prim_path (str): Path to the camera prim. Global path when it starts with `/`, else local to vehicle prim path
            config (dict): A Dictionary that contains all the parameters for configuring the ROS2Camera - it can be empty or only have some of the parameters used by the ROS2Camera.

        Examples:
            The dictionary default parameters are

            >>> {"namespace": "drone"                           # Namespace to append to the topics
            >>>  "pub_pose": True,                              # Publish the pose of the vehicle
            >>>  "pub_twist": True,                             # Publish the twist of the vehicle
            >>>  "pub_twist_inertial": True,                    # Publish the twist of the vehicle in the inertial frame
            >>>  "pub_accel": True,                             # Publish the acceleration of the vehicle
            >>>  "pub_imu": True,                               # Publish the IMU data
            >>>  "pub_mag": True,                               # Publish the magnetometer data
            >>>  "pub_gps": True,                               # Publish the GPS data
            >>>  "pub_gps_vel": True,                           # Publish the GPS velocity data
            >>>  "pose_topic": "state/pose",                    # Position and attitude of the vehicle in ENU
            >>>  "twist_topic": "state/twist",                  # Linear and angular velocities in the body frame of the vehicle
            >>>  "twist_inertial_topic": "state/twist_inertial" # Linear velocity of the vehicle in the inertial frame
            >>>  "accel_topic": "state/accel",                  # Linear acceleration of the vehicle in the inertial frame
            >>>  "imu_topic": "sensors/imu",                    # IMU data
            >>>  "mag_topic": "sensors/mag",                    # Magnetometer data
            >>>  "gps_topic": "sensors/gps",                    # GPS data
            >>>  "gps_vel_topic": "sensors/gps_twist",          # GPS velocity data
            >>>  "pub_graphical_sensors": True,                 # Publish the graphical sensors
            >>>  "pub_sensors": True,                           # Publish the sensors
            >>>  "pub_state": True,                             # Publish the state of the vehicle
            >>>  "pub_tf": False,                               # Publish the TF of the vehicle
            >>>  "sub_control": True,                           # Subscribe to the control topics
            >>>  "use_sim_time": False,                         # Use system time or simulation time
        """

        # Save the configurations for this backend
        self._id = vehicle_id
        self._num_rotors = num_rotors
        self._namespace = config.get("namespace", "drone" + str(vehicle_id))

        # Save what whould be published/subscribed
        self._pub_graphical_sensors = config.get("pub_graphical_sensors", True)
        self._pub_sensors = config.get("pub_sensors", True)
        self._pub_state = config.get("pub_state", True)
        self._sub_control = config.get("sub_control", True)

        # Check if the tf2_ros library is loaded and if the flag is set to True
        self._pub_tf = config.get("pub_tf", False) and tf2_ros_loaded

        # Check if we should use simulation time instead of system time
        self._use_sim_time = config.get("use_sim_time", False)

        # Start the actual ROS2 setup here
        try:
            rclpy.init()
        except:
            # If rclpy is already initialized, just ignore the exception
            pass

        self.node = rclpy.create_node("simulator_vehicle_" + str(vehicle_id))
        
        # Set the use_sim_time parameter if enabled
        if self._use_sim_time:
            self.node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Initialize the publishers and subscribers
        self.initialize_publishers(config)
        self.initialize_subscribers()

        # Create a dictionary that will store the writers for the graphical sensors
        # NOTE: this is done this way, because the writers move data from the GPU->CPU and then publish it to ROS2
        # in a separate thread. This is done to avoid blocking the simulation
        self.graphical_sensors_writers = {}
        
        # Setup zero input reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]

    def initialize(self, vehicle):
        super().initialize(vehicle)

        # -----------------------------------------------------
        # Initialize the static and dynamic tf broadcasters
        # -----------------------------------------------------
        if self._pub_tf:

            # Initiliaze the static tf broadcaster for the sensors
            self.tf_static_broadcaster = StaticTransformBroadcaster(self.node)

            # Initialize the static tf broadcaster for the base_link transformation
            self.send_static_transforms()

            # Initialize the dynamic tf broadcaster for the position of the body of the vehicle (base_link) with respect to the inertial frame (map - ENU) expressed in the inertil frame (map - ENU)
            self.tf_broadcaster = TransformBroadcaster(self.node)
    
    def initialize_publishers(self, config: dict):

        # ----------------------------------------------------- 
        # Create publishers for the state of the vehicle in ENU
        # -----------------------------------------------------
        if self._pub_state:
            if config.get("pub_pose", True):
                self.pose_pub = self.node.create_publisher(PoseStamped, self._namespace + str(self._id) + "/" + config.get("pose_topic", "state/pose"), rclpy.qos.qos_profile_sensor_data)
            
            if config.get("pub_twist", True):
                self.twist_pub = self.node.create_publisher(TwistStamped, self._namespace + str(self._id) + "/" + config.get("twist_topic", "state/twist"), rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_twist_inertial", True):
                self.twist_inertial_pub = self.node.create_publisher(TwistStamped, self._namespace + str(self._id) + "/" + config.get("twist_inertial_topic", "state/twist_inertial"), rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_accel", True):
                self.accel_pub = self.node.create_publisher(AccelStamped, self._namespace + str(self._id) + "/" + config.get("accel_topic", "state/accel"), rclpy.qos.qos_profile_sensor_data)

        # -----------------------------------------------------
        # Create publishers for the sensors of the vehicle
        # -----------------------------------------------------
        if self._pub_sensors:
            if config.get("pub_imu", True):
                self.imu_pub = self.node.create_publisher(Imu, self._namespace + str(self._id) + "/" + config.get("imu_topic", "sensors/imu"), rclpy.qos.qos_profile_sensor_data)
            
            if config.get("pub_mag", True):
                self.mag_pub = self.node.create_publisher(MagneticField, self._namespace + str(self._id) + "/" + config.get("mag_topic", "sensors/mag"), rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_gps", True):
                self.gps_pub = self.node.create_publisher(NavSatFix, self._namespace + str(self._id) + "/" + config.get("gps_topic", "sensors/gps"), rclpy.qos.qos_profile_sensor_data)
            
            if config.get("pub_gps_vel", True):
                self.gps_vel_pub = self.node.create_publisher(TwistStamped, self._namespace + str(self._id) + "/" + config.get("gps_vel_topic", "sensors/gps_twist"), rclpy.qos.qos_profile_sensor_data)
        
        # -----------------------------------------------------
        # Create publisher for the simulation clock
        # -----------------------------------------------------
        self.clock_pub = None
        if self._use_sim_time:
            # Only create clock publisher if no other ROS2Backend instance has created one
            if not ROS2Backend._clock_publisher_created:
                self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
                ROS2Backend._clock_publisher_created = True

    def initialize_subscribers(self):

        if self._sub_control:
            # Subscribe to vector of floats with the target angular velocities to control the vehicle
            # This is not ideal, but we need to reach out to NVIDIA so that they can improve the ROS2 support with custom messages
            # The current setup as it is.... its a pain!!!!
            self.rotor_subs = []
            for i in range(self._num_rotors):
                # FIX: Use default argument to capture loop variable correctly
                self.rotor_subs.append(self.node.create_subscription(Float64, self._namespace + str(self._id) + "/control/rotor" + str(i) + "/ref", lambda x, rotor_id=i: self.rotor_callback(x, rotor_id),10))


    def send_static_transforms(self):

        l = []
        # Create the transformation from base_link FLU (ROS standard) to base_link FRD (standard in airborn and marine vehicles)
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = self._namespace + '_' + 'base_link'
        t.child_frame_id = self._namespace + '_' + 'base_link_frd'

        # Converts from FLU to FRD
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 1.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        #self.tf_static_broadcaster.sendTransform(t)
        l.append(t)

        # Create the transform from map, i.e inertial frame (ROS standard) to map_ned (standard in airborn or marine vehicles)
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "map_ned"
        
        # Converts ENU to NED
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = -0.7071068
        t.transform.rotation.y = -0.7071068
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        #self.tf_static_broadcaster.sendTransform(t)
        l.append(t)

        # Create the transforms for all graphical sensors
        for graphical_sensor in self.vehicle._graphical_sensors:
            gs_position = graphical_sensor.position
            gs_orientation = graphical_sensor.orientation

            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = self._namespace + '_' + 'base_link'
            t.child_frame_id = graphical_sensor.name

            t.transform.translation.x = gs_position[0]
            t.transform.translation.y = gs_position[1]
            t.transform.translation.z = gs_position[2]
            t.transform.rotation.x = gs_orientation[0]
            t.transform.rotation.y = gs_orientation[1]
            t.transform.rotation.z = gs_orientation[2]
            t.transform.rotation.w = gs_orientation[3]

            l.append(t)

        self.tf_static_broadcaster.sendTransform(l)

    def update_state(self, state):
        """
        Method that when implemented, should handle the receivel of the state of the vehicle using this callback
        """

        # Update the dynamic tf broadcaster with the current position of the vehicle in the inertial frame
        if self._pub_tf:
            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = self._namespace + '_' + 'base_link'
            t.transform.translation.x = state.position[0]
            t.transform.translation.y = state.position[1]
            t.transform.translation.z = state.position[2]
            t.transform.rotation.x = state.attitude[0]
            t.transform.rotation.y = state.attitude[1]
            t.transform.rotation.z = state.attitude[2]
            t.transform.rotation.w = state.attitude[3]
            self.tf_broadcaster.sendTransform(t)

        # Publish the state of the vehicle only if the flag is set to True
        if not self._pub_state:
            return

        pose = PoseStamped()
        twist = TwistStamped()
        twist_inertial = TwistStamped()
        accel = AccelStamped()

        # Update the header
        pose.header.stamp = self.node.get_clock().now().to_msg()
        twist.header.stamp = pose.header.stamp
        twist_inertial.header.stamp = pose.header.stamp
        accel.header.stamp = pose.header.stamp

        pose.header.frame_id = "map"
        twist.header.frame_id = self._namespace + "_" + "base_link"
        twist_inertial.header.frame_id = "map"
        accel.header.frame_id = "map"

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

    def rotor_callback(self, ros_msg: Float64, rotor_id):
        # Update the reference for the rotor of the vehicle
        self.input_ref[rotor_id] = float(ros_msg.data)

    def update_sensor(self, sensor_type: str, data):
        """
        Method that when implemented, should handle the receival of sensor data
        """

        # Only process sensor data if the flag is set to True
        if not self._pub_sensors:
            return

        if sensor_type == "IMU":
            self.update_imu_data(data)
        elif sensor_type == "GPS":
            self.update_gps_data(data)
        elif sensor_type == "Magnetometer":
            self.update_mag_data(data)
        else:
            pass

    def update_graphical_sensor(self, sensor_type: str, data):
        """
        Method that when implemented, should handle the receival of graphical sensor data
        """

        # Only process graphical sensor data if the flag is set to True
        if not self._pub_graphical_sensors:
            return

        if sensor_type == "MonocularCamera":
            self.update_monocular_camera_data(data)
        elif sensor_type == "Lidar":
            self.update_lidar_data(data)
        else:
            pass

    def update_imu_data(self, data):

        msg = Imu()

        # Update the header
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self._namespace + '_' + "base_link_frd"
        
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
        msg.header.frame_id = "map_ned"
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
        self.mag_pub.publish(msg)

    def update_monocular_camera_data(self, data):

        # Check if the camera name exists in the writers dictionary
        if data["camera_name"] not in self.graphical_sensors_writers:
            self.add_monocular_camera_writter(data)


    def add_monocular_camera_writter(self, data):

        # List all the available writers: print(rep.writers.WriterRegistry._writers)
        render_prod_path = data["camera"]._render_product_path

        # System time or simulation time
        time_type = ""
        if not self._use_sim_time:
            time_type = "SystemTime"
        # Create the writer for the rgb camera
        writer = rep.writers.get(f"LdrColorSDROS2{time_type}PublishImage")
        writer.initialize(nodeNamespace=self._namespace + str(self._id), topicName=data["camera_name"] + "/color/image_raw", frameId=data["camera_name"], queueSize=1)
        writer.attach([render_prod_path])

        # Add the writer to the dictionary
        self.graphical_sensors_writers[data["camera_name"]] = [writer]

        # Check if depth is enabled, if so, set the depth properties
        if "depth" in data:

            # Create the writer for the depth camera
            writer_depth = rep.writers.get(f"DistanceToImagePlaneSDROS2{time_type}PublishImage")
            writer_depth.initialize(nodeNamespace=self._namespace + str(self._id), topicName=data["camera_name"] + "/depth", frameId=data["camera_name"], queueSize=1)
            writer_depth.attach([render_prod_path])

            # Add the writer to the dictionary
            self.graphical_sensors_writers[data["camera_name"]].append(writer_depth)

        # Create a writer for publishing the camera info
        writer_info = rep.writers.get(f"ROS2{time_type}PublishCameraInfo")
        camera_info, _ = read_camera_info(render_product_path=render_prod_path)
        writer_info.initialize(
            nodeNamespace=self._namespace + str(self._id), 
            topicName=data["camera_name"] + "/color/camera_info", 
            frameId=data["camera_name"], 
            queueSize=1,
            width=camera_info.width,
            height=camera_info.height,
            projectionType=camera_info.distortion_model,
            k=camera_info.k.reshape([1, 9]),
            r=camera_info.r.reshape([1, 9]),
            p=camera_info.p.reshape([1, 12]),
            physicalDistortionModel=camera_info.distortion_model,
            physicalDistortionCoefficients=camera_info.d,
        )

        writer_info.attach([render_prod_path])

        # Add the writer to the dictionary
        self.graphical_sensors_writers[data["camera_name"]].append(writer_info)

        gate_path = omni.syntheticdata.SyntheticData._get_node_path("PostProcessDispatch" + "IsaacSimulationGate", render_prod_path)

        # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
        og.Controller.attribute(gate_path + ".inputs:step").set(int(60/data["frequency"]))

        # Only set the simulation time reset on stop if using sim time
        if self._use_sim_time:
            omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                "IsaacReadSimulationTime", {"inputs:resetOnStop": True}, render_prod_path
            )

    def update_lidar_data(self, data):

        # Check if the lidar name exists in the writers dictionary
        if data["lidar_name"] not in self.graphical_sensors_writers:
            self.add_lidar_writter(data)
    
    def add_lidar_writter(self, data):

        # List all the available writers: print(rep.writers.WriterRegistry._writers)
        render_prod_path = rep.create.render_product(data["stage_prim_path"], [1, 1], name=data["lidar_name"])

        # System time or simulation time
        time_type = ""
        if not self._use_sim_time:
            time_type = "SystemTime"
        # Create the writer for the lidar
        writer = rep.writers.get(f"RtxLidarROS2{time_type}PublishPointCloud")
        writer.initialize(nodeNamespace=self._namespace + str(self._id), topicName=data["lidar_name"] + "/pointcloud", frameId=data["lidar_name"])
        writer.attach([render_prod_path])

        # Add the writer to the dictionary
        self.graphical_sensors_writers[data["lidar_name"]] = [writer]

        # Only add LaserScan writer for 2D Lidar
        if data["number_of_emitters"] == 1:
            # Create the writer for publishing a laser scan message along with the point cloud
            writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
            writer.initialize(nodeNamespace=self._namespace + str(self._id), topicName=data["lidar_name"] + "/laserscan", frameId=data["lidar_name"])
            writer.attach([render_prod_path])
            self.graphical_sensors_writers[data["lidar_name"]].append(writer)

        # Only set the simulation time reset on stop if using sim time
        if self._use_sim_time:
            omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                "IsaacReadSimulationTime", {"inputs:resetOnStop": True}, render_prod_path.path
            )

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

        # Publish the simulation time to /clock topic (only if this backend owns the clock publisher)
        if self.clock_pub is not None and self.vehicle is not None and hasattr(self.vehicle, '_world'):
            # Get the simulation time directly from Isaac Sim's World
            sim_time = self.vehicle._world.current_time
            sec = int(sim_time)
            nanosec = int((sim_time - sec) * 1e9)
            
            # Publish the simulation time to /clock topic
            clock_msg = Clock()
            clock_msg.clock = Time(seconds=sec, nanoseconds=nanosec, clock_type=ClockType.ROS_TIME).to_msg()
            self.clock_pub.publish(clock_msg)

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
        
        # Reset the clock publisher flag if this instance owns the clock publisher
        if self.clock_pub is not None:
            ROS2Backend._clock_publisher_created = False

    def reset(self):
        """
        Method that when implemented, should handle the reset of the vehicle simulation to its original state
        """
        # Reset the reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]
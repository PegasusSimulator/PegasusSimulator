"""
| File: imu.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Simulates a monocular camera attached to the vehicle
"""
__all__ = ["MonocularCamera"]

from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.graphical_sensors import GraphicalSensor
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Camera interface provided by NVidia Isaac Sim
import omni
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd

from omni.isaac.sensor import Camera
from omni.usd import get_stage_next_free_path

# Auxiliary scipy and numpy modules
import numpy as np
from scipy.spatial.transform import Rotation

# Replicator imports
import omni.kit
import omni.usd
import omni.replicator.core as rep
from omni.replicator.core import Writer, AnnotatorRegistry

# ROS2 imports
import rclpy
from sensor_msgs.msg import Image, Imu

import multiprocessing as mp
from rclpy.executors import MultiThreadedExecutor

class GraphicalWorkerWriter(Writer):
    
    def __init__(self, rgb: bool = True):
        
        self.annotators = []

        # RGB
        if rgb:
            self.annotators.append(AnnotatorRegistry.get_annotator("rgb"))

        # Process that will actually be publishing the image data
        #self.queue = mp.Queue()
        # p1 = mp.Process(target=self.publish_image)
        # p1.start()
        # print("Starting worker process")

    # def publish_image(self):
        
    #     # Initialize the ROS2 node
    #     #rclpy.init()
    #     node = rclpy.create_node("vehicle")
    #     image_pub = node.create_publisher(Image, "/camera/image", 10)
    #     imu_pub = node.create_publisher(Imu, "/imu/data", 10)

    #     executor = MultiThreadedExecutor(num_threads=2)
    #     executor.add_node(node)


    #     while True:

    #         # Get the image that will be published
    #         image = self.queue.get()

    #         msg = Image()
    #         msg.header.frame_id = "camera"
    #         msg.header.stamp = node.get_clock().now().to_msg()
    #         msg.data = image.tobytes()
    #         msg.step = 3 * 1920
    #         msg.height = 1280
    #         msg.width = 1920
    #         msg.encoding = "rgb8"

    #         #image_pub.publish(msg)

    #         msg2 = Imu()
    #         msg2.header.frame_id = "imu"

    #         imu_pub.publish(msg2)

    #         rclpy.spin_once(node, executor=executor, timeout_sec=0)
            
    #         print("Publishing image")


    def write(self, data):

        if "rgb" in data:
            pass
            #self.queue.put(data["rgb"])
            #print("Writing image")

            

# Register this writer in the replicator registry
rep.WriterRegistry.register(GraphicalWorkerWriter)

class MonocularCamera(GraphicalSensor):
    """
    The class that implements a monocular camera sensor. This class inherits the base class GraphicalSensor.
    """

    def __init__(self, camera_name, config={}):
        """
        Initialize the MonocularCamera class
        
        Check the oficial documentation for the Camera class in Isaac Sim: 
        https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_camera.html#isaac-sim-sensors-camera

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the MonocularCamera - it can be empty or only have some of the parameters used by the MonocularCamera.

        Examples:
            The dictionary default parameters are

            >>> {"focal_length": 0.004,
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="MonocularCamera", update_rate=config.get("frequency", 60.0))        
        
        # Setup the name of the camera primitive path
        self._camera_name = camera_name
        self._stage_prim_path = ""

        # Configurations of the camera
        self._position = config.get("position", np.array([0.0, 0.0, 0.0]))
        self._orientation = config.get("orientation", np.array([0.0, 0.0, 0.0]))
        self._resolution = config.get("resolution", (1920, 1200))
        self._frequency = config.get("frequency", 30)
        self._intrinsics = config.get("intrinsics", np.array([[958.8, 0.0, 957.8], [0.0, 956.7, 589.5], [0.0, 0.0, 1.0]]))
        self._distortion_coefficients = config.get("distortion_coefficients", np.array([0.14, -0.03, -0.0002, -0.00003, 0.009, 0.5, -0.07, 0.017]))
        self._diagonal_fov = config.get("diagonal_fov", 140.0)

        # Setup an empty camera output dictionary
        self._state = {}


    def initialize(self, vehicle):
        
        # Initialize the Super class "object" attributes
        super().initialize(vehicle)

        # Get the complete stage prefix for the camera
        self._stage_prim_path = get_stage_next_free_path(PegasusInterface().world.stage, self._vehicle.prim_path + "/body/" + self._camera_name, False)

        # Get the camera name that was actually created (and update the camera name)
        self._camera_name = self._stage_prim_path.rpartition("/")[-1]

        # Create the camera object attached to the rigid body vehicle
        self._camera = Camera(
            prim_path=self._stage_prim_path,
            position=np.array(self._position),
            frequency=self._frequency,
            resolution=self._resolution,
            orientation=Rotation.from_euler("XYZ", self._orientation, degrees=True).as_quat())
        
    def start(self):

        # Set the camera intrinsics
        ((fx,_,cx),(_,fy,cy),(_,_,_)) = self._intrinsics

        # Start the camera
        self._camera.initialize()

        # Set the correct properties of the camera (this must be done after the camera object is initialized)
        self._camera.set_projection_type("fisheyePolynomial")  # # f-theta model, to approximate the fisheye model
        self._camera.set_rational_polynomial_properties(self._resolution[0], self._resolution[1], cx, cy, self._diagonal_fov, self._distortion_coefficients)

        # Start the ROS 2 writter
        render_prod_path = rep.create.render_product(self._stage_prim_path, resolution=(self._resolution[0], self._resolution[1]))

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)

        #writer = rep.writers.get("BasicWriter")
        #writer.initialize(output_dir="/home/marcelo/aaa", rgb=True)
        
        writer = rep.writers.get("GraphicalWorkerWriter")
        writer.initialize(rgb=True)

        writer.attach([render_prod_path])

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state


    @GraphicalSensor.update_at_rate
    def update(self, state: State, dt: float):
        """Method that gets the current RGB image from the camera and returns it as a dictionary.

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        # Get the data from the camera
        try:
            self._state = {}
            self._state["camera_name"] = self._camera_name
            self._state["image"] = self._camera.get_rgba()[:, :, :3]
            self._state["height"] = self._resolution[1]
            self._state["width"] = self._resolution[0]

            if self._camera.get_projection_type() == "pinhole":
                self._state["intrinsics"] = self._camera.get_intrinsics_matrix()
            
        # If something goes wrong during the data acquisition, just return None
        except:
            self._state = None

        return self._state

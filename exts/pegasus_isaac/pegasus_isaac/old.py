
# OLD SNIPETS OF CODE THAT WORK

# -------------------------------------
# Adding a robot to the stage
# -------------------------------------

# Tell this stage that a USD model of a drone exists and where it is "inside the file"

#add_reference_to_stage(usd_path=ROBOTS["Quadrotor"], prim_path="/World/quadrotor")

# Create the a "robot" object wrapper around the "/World/quadrotor" primitive
#self.robot = Robot(
#   prim_path="/World/quadrotor",
#   position=np.array([0.0, 0.0, 1.0]),
#   articulation_controller=None)

# Add the drone USD model to the scene
#self._world.scene.add(self.robot)

#self._world.scene.add()
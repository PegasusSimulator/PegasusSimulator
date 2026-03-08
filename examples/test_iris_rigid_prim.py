#!/usr/bin/env python
"""
| File: 4_python_single_vehicle.py
| Author: Marcelo Jacinto and Joao Pinto (marcelo.jacinto@tecnico.ulisboa.pt, joao.s.pinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to use the control backends API to create a custom controller 
for the vehicle from scratch and use it to perform a simulation, without using PX4 nor ROS.
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": True})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

import numpy as np
import torch
from pxr import UsdGeom, Gf, UsdLux, PhysxSchema

import isaacsim.core.utils.stage as stage_utils
#from isaacsim.core.prims import Articulation
from isaacsim.core.prims import RigidPrim

USD_PATH = "/home/rodrigogomes/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

class PegasusApp:
    """
            forces = torch.zeros((self.n_envs, self.num_children, 3), device=self.device, dtype=torch.float32)
            forces[:, 1:, 2] = float(0.005) 
            forces = forces.reshape((self.prims.count, 3))

            torques = torch.zeros((self.n_envs, self.num_children, 3), device=self.device, dtype=torch.float32)
            torques[:, 0, 2] = float(0.01) 
            torques = torques.reshape((self.prims.count, 3))

            #print("Applying forces:", forces)
            #print("Applying torques:", torques)

            self.prims.apply_forces_and_torques_at_pos(forces, torques, is_global=False)

            # Update the UI of the app and perform the physics step
            self.pg._world.step(render=True)

            i += 1
            print("Step:", i)
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self, n_envs=8, spacing=3.0):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        #self.pg._world = World(**self.pg._world_settings)
        self.pg._world = World(
            physics_dt = 1.0 / 100.0,
            stage_units_in_meters=1.0,
            rendering_dt=1.0 / 60.0,
            device="cuda"
        )

        stage = stage_utils.get_current_stage()
        scenePrim = stage.GetPrimAtPath("/physicsScene")  # adjust to your scene path
        print(scenePrim)
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scenePrim)
        physxSceneAPI.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(24576)        

        self.pg._world.get_physics_context().enable_gpu_dynamics(True)
        print(self.pg._world.get_physics_context().is_gpu_dynamics_enabled())

        
        #UsdGeom.Xform.Define(stage, "/World")

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        self.n_envs = n_envs

        # Spawn N IRIS
        side = int(np.ceil(np.sqrt(self.n_envs)))
        for i in range(self.n_envs):
            prim_path = f"/World/quadrotor{i}"
            stage_utils.add_reference_to_stage(USD_PATH, prim_path=prim_path)

            x = (i % side) * spacing
            y = (i // side) * spacing
            z = 0.30

            prim = stage.GetPrimAtPath(prim_path)
            xformable = UsdGeom.XformCommonAPI(UsdGeom.Xformable(prim))
            xformable.SetTranslate(Gf.Vec3d(float(x), float(y), float(z)))
            rotate_op = prim.GetAttribute("xformOp:rotateXYZ")
            rotate_op.Set(Gf.Vec3d(45.0, 0.0, 0.0))

            self.num_children = len(prim.GetChildren())

        self.pg._world.get_physics_context().set_gravity(0.0)
        self.pg._world.reset()

        # Bodies (for mass estimate)
        self.prims = RigidPrim(prim_paths_expr="/World/quadrotor.*/.*", name="prims")
        self.prims.initialize()

        print("prims.count =", self.prims.count)

        #print("prims.prim_paths =", self.prims.prim_paths)

        self.device = torch.device("cuda")


    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        i = 0
        # The "infinite" loop
        while simulation_app.is_running():

            # One force vector per rotor rigid body => shape (N*4, 3)
            forces = torch.zeros((self.n_envs, self.num_children, 3), device=self.device, dtype=torch.float32)
            forces[:, 1:, 2] = float(0.005) 
            forces = forces.reshape((self.prims.count, 3))

            torques = torch.zeros((self.n_envs, self.num_children, 3), device=self.device, dtype=torch.float32)
            torques[:, 0, 2] = float(0.01) 
            torques = torques.reshape((self.prims.count, 3))

            #print("Applying forces:", forces)
            #print("Applying torques:", torques)

            self.prims.apply_forces_and_torques_at_pos(forces, torques, is_global=False)

            # Update the UI of the app and perform the physics step
            self.pg._world.step(render=False, step_sim=True)

            i += 1
            print("Step:", i)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    pg_app = PegasusApp(n_envs=4096, spacing=3.0)

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
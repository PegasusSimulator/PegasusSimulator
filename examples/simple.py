#!/usr/bin/env python

"""
This example file shows how to initialize a simulation using the Pegasus framework directly from a python script
"""
from omni.isaac.kit import SimulationApp
from omni.isaac.core.simulation_context import SimulationContext
from pegasus_isaac.logic.pegasus_simulator import PegasusSimulator

# Start Isaac Sim's simulation environment
simulation_app = SimulationApp({"headless": False})

# Start the Pegasus Simulator backend
pg_sim = PegasusSimulator()

# get simulation context
simulation_context = SimulationContext()
# rest and play simulation
simulation_context.reset()
# step simulation
simulation_context.step()
# stop simulation
simulation_context.stop()

# Close the Isaac Sim simulator
simulation_app.close()

#!/usr/bin/env python
"""
This example file demonstrates how to create custom backends that allow for controling and receiving simulation data from
the Pegasus simulator without necessarily using PX4 or ROS2
"""
from omni.isaac.kit import SimulationApp
from omni.isaac.core.simulation_context import SimulationContext
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

def main():
    """
    Defines the main simulation to perform using Isaac Sim
    """

    # Start the Pegasus Simulator backend
    pg_sim = PegasusInterface()

    # get simulation context
    simulation_context = SimulationContext()
    # rest and play simulation
    simulation_context.reset()
    # step simulation
    simulation_context.step()
    # stop simulation
    simulation_context.stop()


if __name__ == "__main__":

    # Start Isaac Sim's simulation environment
    simulation_app = SimulationApp({"headless": False})

    # Execute the main simulation code
    main()

    # Close the Isaac Sim simulator
    simulation_app.close()

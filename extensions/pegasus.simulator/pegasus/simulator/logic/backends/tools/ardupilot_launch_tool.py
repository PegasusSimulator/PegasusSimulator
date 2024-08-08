"""
| File: ardupilot_launch_tool.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: Defines an auxiliary tool to launch the Ardupilot process in the background
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""

# System tools used to launch the ardupilot process in the brackground
import os
import time
import tempfile
import subprocess


class ArdupilotLaunchTool:
    """
    A class that manages the start/stop of a ardupilot process. It requires only the path to the Ardupilot installation (assuming that
    Ardupilot was already built with 'make ardupilot_sitl_default none'), the vehicle id and the vehicle model. 
    """

    def __init__(self, ardupilot_dir, vehicle_id: int = 0, ardupilot_model: str = "gazebo-iris"):
        """Construct the ArdupilotLaunchTool object

        Args:
            ardupilot_dir (str): A string with the path to the Ardupilot-Autopilot directory
            vehicle_id (int): The ID of the vehicle. Defaults to 0.
            ardupilot_model (str): The vehicle model. Defaults to "iris".
        """

        # Attribute that will hold the ardupilot process once it is running
        self.ardupilot_process = None

        # The vehicle id (used for the mavlink port open in the system)
        self.vehicle_id = vehicle_id

        # Configurations to whether autostart ardupilot (SITL) automatically or have the user launch it manually on another
        # terminal
        self.ardupilot_dir = ardupilot_dir

        # Create a temporary filesystem for ardupilot to write data to/from (and modify the origin rcS files)
        self.root_fs = tempfile.TemporaryDirectory()

        # Set the environement variables that let Ardupilot know which vehicle model to use internally
        self.environment = os.environ

    def launch_ardupilot(self):
        """
        Method that will launch a ardupilot instance with the specified configuration
        """
        self.ardupilot_process = subprocess.Popen(
            [
                self.ardupilot_dir + "/Tools/autotest/sim_vehicle.py",
                "-v",
                "ArduCopter",
                "-f",
                "quad",
                f"{'--no-rebuild' if os.path.exists(f'{self.ardupilot_dir}/build/sitl/bin/arducopter') else ''}",
            ],
            cwd=self.root_fs.name,
            shell=False,
            env=self.environment,
            # stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )

    def kill_ardupilot(self):
        """
        Method that will kill a ardupilot instance with the specified configuration
        """
        if self.ardupilot_process is not None:
            self.ardupilot_process.kill()
            self.ardupilot_process = None

    def __del__(self):
        """
        If the ardupilot process is still running when the Ardupilot launch tool object is whiped from memory, then make sure
        we kill the ardupilot instance so we don't end up with hanged ardupilot instances
        """

        # Make sure the Ardupilot process gets killed
        if self.ardupilot_process:
            self.kill_ardupilot()

        # Make sure we clean the temporary filesystem used for the simulation
        self.root_fs.cleanup()


# ---- Code used for debugging the ardupilot tool ----
def main():

    # TODO
    ardupilot_dir = f"{os.environ['HOME']}/dev/PegasusSimulator/ardupilot"

    ardupilot_tool = ArdupilotLaunchTool(ardupilot_dir=ardupilot_dir)
    ardupilot_tool.launch_ardupilot()
    
    time.sleep(60)


if __name__ == "__main__":
    main()

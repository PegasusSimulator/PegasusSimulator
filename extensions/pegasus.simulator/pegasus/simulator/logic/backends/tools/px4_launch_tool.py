"""
| File: px4_launch_tool.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: Defines an auxiliary tool to launch the PX4 process in the background
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""

# System tools used to launch the px4 process in the brackground
import os
import tempfile
import subprocess


class PX4LaunchTool:
    """
    A class that manages the start/stop of a px4 process. It requires only the path to the PX4 installation (assuming that
    PX4 was already built with 'make px4_sitl_default none'), the vehicle id and the vehicle model. 
    """

    def __init__(self, px4_dir, vehicle_id: int = 0, px4_model: str = "gazebo-classic_iris"):
        """Construct the PX4LaunchTool object

        Args:
            px4_dir (str): A string with the path to the PX4-Autopilot directory
            vehicle_id (int): The ID of the vehicle. Defaults to 0.
            px4_model (str): The vehicle model. Defaults to "iris".
        """

        # Attribute that will hold the px4 process once it is running
        self.px4_process = None

        # The vehicle id (used for the mavlink port open in the system)
        self.vehicle_id = vehicle_id

        # Configurations to whether autostart px4 (SITL) automatically or have the user launch it manually on another
        # terminal
        self.px4_dir = px4_dir
        self.rc_script = self.px4_dir + "/ROMFS/px4fmu_common/init.d-posix/rcS"

        # Create a temporary filesystem for px4 to write data to/from (and modify the origin rcS files)
        self.root_fs = tempfile.TemporaryDirectory()

        # Set the environement variables that let PX4 know which vehicle model to use internally
        self.environment = os.environ
        self.environment["PX4_SIM_MODEL"] = px4_model

    def launch_px4(self):
        """
        Method that will launch a px4 instance with the specified configuration
        """
        self.px4_process = subprocess.Popen(
            [
                self.px4_dir + "/build/px4_sitl_default/bin/px4",
                self.px4_dir + "/ROMFS/px4fmu_common/",
                "-s",
                self.rc_script,
                "-i",
                str(self.vehicle_id),
                "-d",
            ],
            cwd=self.root_fs.name,
            shell=False,
            env=self.environment,
        )

    def kill_px4(self):
        """
        Method that will kill a px4 instance with the specified configuration
        """
        if self.px4_process is not None:
            self.px4_process.kill()
            self.px4_process = None

    def __del__(self):
        """
        If the px4 process is still running when the PX4 launch tool object is whiped from memory, then make sure
        we kill the px4 instance so we don't end up with hanged px4 instances
        """

        # Make sure the PX4 process gets killed
        if self.px4_process:
            self.kill_px4()

        # Make sure we clean the temporary filesystem used for the simulation
        self.root_fs.cleanup()


# ---- Code used for debugging the px4 tool ----
def main():

    px4_tool = PX4LaunchTool(os.environ["HOME"] + "/PX4-Autopilot")
    px4_tool.launch_px4()

    import time

    time.sleep(60)


if __name__ == "__main__":
    main()

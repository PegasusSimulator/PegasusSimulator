import omni.kit.app as app
from pathlib import Path

# Extension configuration
EXTENSION_NAME = "Pegasus Isaac"
WINDOW_TITLE = "Pegasus Isaac"
DOC_LINK = ("https://docs.omniverse.nvidia.com")
EXTENSION_OVERVIEW = "This extension shows how to incorporate drones into Isaac Sim"

# Get the current directory of where this extension is located
EXTENSION_FOLDER_PATH = Path(app.get_app().get_extension_manager().get_extension_path_by_module(__name__))
ROOT = str(EXTENSION_FOLDER_PATH.parent.parent.resolve())

# Define the Extension Assets Path
ASSET_PATH = ROOT + "/exts/pegasus_isaac/pegasus_isaac/assets/"
ROBOTS_ASSETS = ASSET_PATH + "/Robots"

# Define the built in robots of the extension
ROBOTS = {
    "Quadrotor": ROBOTS_ASSETS + "/iris_base.usda",
    "Iris": ROBOTS_ASSETS + "/iris_base.usda",
    "Iris FPV": ROBOTS_ASSETS + "/iris_base.usda"
}

SIMULATION_ENVIRONMENTS = {
    "Empty World": "/",
    "Forest": "/", 
}

# Define the default settings for the simulation environment
DEFAULT_WORLD_SETTINGS = {
    "physics_dt": 1.0 / 250.0, 
    "stage_units_in_meters": 1.0, 
    "rendering_dt": 1.0 / 60.0
}
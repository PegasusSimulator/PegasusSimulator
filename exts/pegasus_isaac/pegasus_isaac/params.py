from pathlib import Path

import omni.kit.app as app
import omni.isaac.core.utils.nucleus as nucleus

# Extension configuration
EXTENSION_NAME = "Pegasus Simulator"
WINDOW_TITLE = "Pegasus Simulator"
MENU_PATH = "Window/" + WINDOW_TITLE
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
    "Iris": ROBOTS_ASSETS + "/Iris/iris.usda",
    "Flying Cube": ROBOTS_ASSETS + "/iris_cube.usda"
}

# Setup the default simulation environments path
NVIDIA_ASSETS_PATH = nucleus.get_assets_root_path()
ISAAC_SIM_ENVIRONMENTS = "/Isaac/Environments"
NVIDIA_SIMULATION_ENVIRONMENTS = {
    "Default Environment": "Grid/default_environment.usd",
    "Black Gridroom": "Grid/gridroom_black.usd",
    "Curved Gridroom": "Grid/gridroom_curved.usd",
    "Hospital": "Hospital/hospital.usd",
    "Office": "Office/office.usd",
    "Simple Room": "Simple_Room/simple_room.usd",
    "Warehouse": "Simple_Warehouse/warehouse.usd",
    "Warehouse with Forklifts": "Simple_Warehouse/warehouse_with_forklifts.usd",
    "Warehouse with Shelves": "Simple_Warehouse/warehouse_multiple_shelves.usd",
    "Full Warehouse": "Simple_Warehouse/full_warehouse.usd",
    "Flat Plane": "Terrains/flat_plane.usd",
    "Rough Plane": "Terrains/rough_plane.usd",
    "Slope Plane": "Terrains/slope.usd",
    "Stairs Plane": "Terrains/stairs.usd",
}

SIMULATION_ENVIRONMENTS = {}
for asset in NVIDIA_SIMULATION_ENVIRONMENTS:
    SIMULATION_ENVIRONMENTS[asset] = NVIDIA_ASSETS_PATH + ISAAC_SIM_ENVIRONMENTS + '/' + NVIDIA_SIMULATION_ENVIRONMENTS[asset]

# Define the default settings for the simulation environment
DEFAULT_WORLD_SETTINGS = {
    "physics_dt": 1.0 / 250.0, 
    "stage_units_in_meters": 1.0, 
    "rendering_dt": 1.0 / 60.0
}

# Define where the thumbnail of the vehicle is located
THUMBNAIL = ROBOTS_ASSETS + "/Iris/iris_thumbnail.png"

# Define where the thumbail of the world is located
WORLD_THUMBNAIL = ASSET_PATH + "/Worlds/Empty_thumbnail.png"